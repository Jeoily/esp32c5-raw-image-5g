"""RGB reconstruction from captured radio bytes; source pixels are diagnostics only."""
import struct
import zlib

from PIL import Image, ImageOps

MAGIC = b"C5IMG001"
CHUNK = 240
HEADER_SIZE = 32
FRAME_HEADER = 24 + HEADER_SIZE
FIRMWARE = "raw_image_v1"
MISSING_COLOR = (32, 36, 44)

# 802.11a L-SIG RATE field -> Mbps. These are the standard 20 MHz values;
# on the C5 the field is documented as "Rate field of L-SIG" and is reported
# as raw metadata, so the mapping is an inference, not a driver guarantee.
L_SIG_RATE_MBPS = {0xB: 6, 0xF: 9, 0xA: 12, 0xE: 18,
                   0x9: 24, 0xD: 36, 0x8: 48, 0xC: 54}

BER_SCOPE = ("仅统计已提交、已收到且应用头CRC通过的载荷位；"
             "不含丢包、坏头、截断及FCS失败后被拒绝归属的帧，存在接收选择偏差")


def command_record(body):
    encoded = body.encode("ascii")
    return b"!" + encoded + f"*{zlib.crc32(encoded):08x}\n".encode("ascii")


def parse_record(line):
    if not line.startswith("!"):
        return None
    body, checksum = line[1:].rsplit("*", 1)
    if len(checksum) != 8 or zlib.crc32(body.encode("ascii")) != int(checksum, 16):
        raise ValueError("串口记录CRC不一致")
    return body.split()


def prepare_image(path, max_width=256):
    if max_width not in (160, 256, 320, 640):
        raise ValueError("图像上限请选择160、256、320或640。")
    with Image.open(path) as source:
        image = ImageOps.exif_transpose(source).convert("RGB")
        image.thumbnail((max_width, max_width * 3 // 4), Image.Resampling.LANCZOS)
    return image, image.tobytes()


def encode_frame(run, sequence, width, height, source_mac, payload):
    """Reference encoder for offline checks; firmware implements the same layout."""
    header = struct.pack("<8sIIHHHHI", MAGIC, run, sequence, width, height,
                         len(payload), len(payload) ^ 65535, 0)
    mac = bytearray(24)
    mac[0] = 8
    mac[4:10] = b"\xff" * 6
    mac[10:16] = bytes.fromhex(source_mac)
    mac[16:22] = bytes.fromhex(source_mac)
    return bytes(mac) + header + struct.pack("<I", zlib.crc32(header)) + payload


def rx_phy(fields):
    """Optional trailing PHY metadata. Old 10-field records yield an empty
    dict, so archived results stay readable with the new host code."""
    if len(fields) < 16:
        return {}
    try:
        rate, bb_format, second, noise_floor, he_siga1, local_hint = (
            int(value) for value in fields[10:16])
    except ValueError:
        return {}
    return dict(rate=rate, bb_format=bb_format, second=second,
                noise_floor=noise_floor, he_siga1=he_siga1, local_hint=local_hint)


class ImageReceiver:
    """No source image is available here. A missing block remains missing."""
    def __init__(self, run, source_mac):
        self.run = run
        self.source_mac = bytes.fromhex(source_mac)
        self.size = None
        self.pixels = bytearray()
        self.frames = {}
        self.duplicates = 0
        self.unidentified = 0
        self.header_corrupt = 0
        self.truncated = 0
        self.inconsistent = 0
        self.state_histogram = {}
        # 档1: count every frame the firmware considered local, including ones
        # rejected by the strict header gate of the primary image.
        self.local_frames = 0
        self.local_fcs_fail_frames = 0
        self.phy_rate_histogram = {}
        self.phy_rate_mbps_histogram = {}
        self.phy_bb_format_histogram = {}
        self.phy_second_histogram = {}
        self.phy_noise_floor_total = 0
        self.phy_metadata_samples = 0
        self.foreign_weak_candidates = 0
        # 档3: best-effort payloads recovered from FCS-failed frames.
        self.raw_frames = {}
        self.raw_hints = {}
        self.lenient_size = None
        self.dimension_mismatch_lenient = False

    @staticmethod
    def _bump(histogram, key):
        text = str(key)
        histogram[text] = histogram.get(text, 0) + 1

    def _record_phy(self, phy):
        if not phy:
            return
        self.phy_metadata_samples += 1
        rate = phy['rate']
        self._bump(self.phy_rate_histogram, rate)
        self._bump(self.phy_rate_mbps_histogram, L_SIG_RATE_MBPS.get(rate, 'unknown'))
        self._bump(self.phy_bb_format_histogram, phy['bb_format'])
        self._bump(self.phy_second_histogram, phy['second'])
        self.phy_noise_floor_total += phy['noise_floor']
        if phy['local_hint'] & 8:
            self.foreign_weak_candidates += 1

    def _lenient_block(self, raw, state, phy):
        """Recover (sequence, payload) from a frame whose FCS failed.

        Deliberately bypasses the header CRC and magic checks used by the main
        image. Only used to draw raw_error.png; it never fills or repairs the
        primary received image. A matching run id plus sane size, sequence and
        length are the only guards, so the result is a diagnostic, not data.
        """
        if state == 0 or len(raw) < FRAME_HEADER:
            return
        try:
            run, sequence, width, height, length, inverted, flags = struct.unpack_from(
                "<IIHHHHI", raw, 32)
        except struct.error:
            return
        if run != self.run or not 1 <= width <= 640 or not 1 <= height <= 480:
            return
        if flags != 0 or not 1 <= length <= CHUNK or length > len(raw) - FRAME_HEADER:
            return
        total = width * height * 3
        offset = sequence * CHUNK
        if offset >= total or length > total - offset:
            return
        if self.lenient_size is None:
            self.lenient_size = (width, height)
        elif self.lenient_size != (width, height):
            self.dimension_mismatch_lenient = True
            return
        if sequence not in self.raw_frames:
            self.raw_frames[sequence] = raw[FRAME_HEADER:FRAME_HEADER + length]
            self.raw_hints[sequence] = phy.get('local_hint', 0)

    def feed(self, fields):
        if len(fields) < 10 or fields[0] != "RX":
            raise ValueError("RX记录格式错误")
        (_, timestamp, kind, state, rssi, channel, sig_len, dump_len,
         copied, raw_hex) = fields[:10]
        phy = rx_phy(fields)
        raw = b"" if raw_hex == "-" else bytes.fromhex(raw_hex)
        if len(raw) != int(copied):
            raise ValueError("RX字节长度不符")
        state = int(state)
        self.state_histogram[str(state)] = self.state_histogram.get(str(state), 0) + 1
        # Firmware-side filtering means every RX record that reaches the host
        # was classified as local (strongly or weakly). Count before the strict
        # gate so rejected headers still appear in the layered statistics.
        self.local_frames += 1
        if state:
            self.local_fcs_fail_frames += 1
        self._record_phy(phy)
        self._lenient_block(raw, state, phy)

        if int(channel) != 40 or int(kind) != 2 or len(raw) < 56 or raw[10:16] != self.source_mac:
            self.unidentified += 1
            return None
        if raw[0] != 8 or raw[1] & 0x43 or raw[24:32] != MAGIC:
            self.unidentified += 1
            return None
        header = raw[24:56]
        if zlib.crc32(header[:28]) != struct.unpack_from("<I", header, 28)[0]:
            self.header_corrupt += 1
            return None
        _, run, sequence, width, height, length, inverted, flags = struct.unpack("<8sIIHHHHI", header[:28])
        if run != self.run or not 1 <= width <= 640 or not 1 <= height <= 480 or flags != 0:
            self.unidentified += 1
            return None
        total = width * height * 3
        offset = sequence * CHUNK
        if offset >= total or length != min(CHUNK, total - offset) or inverted != (length ^ 65535):
            self.inconsistent += 1
            return None
        if len(raw) < 56 + length:
            self.truncated += 1
            return None
        if self.size is not None and self.size != (width, height):
            self.inconsistent += 1
            return None
        if sequence in self.frames:
            self.duplicates += 1
            return None
        if self.size is None:
            self.size = (width, height)
            self.pixels = bytearray(bytes(MISSING_COLOR) * (width * height))
        payload = raw[56:56 + length]
        self.pixels[offset:offset + length] = payload
        frame = dict(sequence=sequence, timestamp=int(timestamp), state=state, rssi=int(rssi),
                     length=length, sig_len=int(sig_len), dump_len=int(dump_len),
                     payload=payload, phy=phy)
        self.frames[sequence] = frame
        return frame

    def image(self):
        if self.size is None:
            return None
        return Image.frombytes("RGB", self.size, bytes(self.pixels))

    def raw_error_image(self):
        """Primary received image with FCS-failed raw payloads overlaid.

        Blocks whose headers were too damaged to pass the strict gate appear
        here as their original (possibly wrong-coloured) bytes. Everything
        else is left exactly as received, so this image never repairs data.
        """
        if not self.raw_frames:
            return None
        size = self.size or self.lenient_size
        if size is None:
            return None
        pixels = bytearray(self.pixels) if self.size else bytearray(
            bytes(MISSING_COLOR) * (size[0] * size[1]))
        for sequence, payload in self.raw_frames.items():
            offset = sequence * CHUNK
            if offset + len(payload) <= len(pixels):
                pixels[offset:offset + len(payload)] = payload
        return Image.frombytes("RGB", size, bytes(pixels))

    def raw_error_mask(self):
        """White = block came from a FCS-failed frame shown in raw_error.png."""
        size = self.size or self.lenient_size
        if size is None or not self.raw_frames:
            return None
        mask = bytearray(b"\x00" * (size[0] * size[1]))
        for sequence, payload in self.raw_frames.items():
            start = sequence * CHUNK // 3
            end = min(start + len(payload) // 3, len(mask))
            if start < end:
                mask[start:end] = b"\xff" * (end - start)
        return Image.frombytes("L", size, bytes(mask))

    def missing_mask(self):
        if self.size is None:
            return None
        mask = bytearray(b"\xff" * (self.size[0] * self.size[1]))
        for sequence, frame in self.frames.items():
            start = sequence * CHUNK // 3
            mask[start:start + frame["length"] // 3] = b"\x00" * (frame["length"] // 3)
        return Image.frombytes("L", self.size, bytes(mask))


class ImageRun:
    def __init__(self, run, source_mac, source_image):
        self.run = run
        self.source = source_image.tobytes()
        self.size = source_image.size
        self.receiver = ImageReceiver(run, source_mac)
        self.count = (len(self.source) + CHUNK - 1) // CHUNK
        self.submitted = set()
        self.attempted = set()
        self.tx_errors = 0
        self.comparisons = {}
        self.started = False
        self.sender_complete = False
        self.dimension_mismatch = False

    def receive(self, fields):
        frame = self.receiver.feed(fields)
        if frame is None:
            return None
        if self.receiver.size != self.size:
            self.dimension_mismatch = True
            return frame
        offset = frame["sequence"] * CHUNK
        expected = self.source[offset:offset + frame["length"]]
        bits = sum((a ^ b).bit_count() for a, b in zip(expected, frame["payload"]))
        pixels = sum(expected[i:i+3] != frame["payload"][i:i+3] for i in range(0, len(expected), 3))
        self.comparisons[frame["sequence"]] = dict(bit_errors=bits, pixel_errors=pixels,
            bits=frame["length"] * 8, state=frame["state"])
        return frame

    def summary(self):
        frames = self.receiver.frames
        eligible = [self.comparisons[seq] for seq in self.submitted if seq in self.comparisons]
        bits = sum(item["bits"] for item in eligible)
        errors = sum(item["bit_errors"] for item in eligible)
        received_bytes = sum(frame["length"] for frame in frames.values())
        local = self.receiver.local_frames
        fcs_fail = self.receiver.local_fcs_fail_frames
        api_accepted = len(self.submitted)
        per = ((api_accepted - len(self.submitted & frames.keys())) / api_accepted
               if api_accepted else None)
        return dict(run=self.run, width=self.size[0], height=self.size[1], total_blocks=self.count,
                    sender_complete=self.sender_complete, attempted=len(self.attempted),
                    api_accepted=api_accepted, tx_api_errors=self.tx_errors,
                    unique_received=len(frames), matched_accepted=len(self.submitted & frames.keys()),
                    received_bytes=received_bytes, total_bytes=len(self.source),
                    missing_blocks=self.count - len(frames),
                    unmatched_accepted=len(self.submitted - frames.keys()),
                    payload_error_frames=sum(item["bit_errors"] > 0 for item in eligible),
                    error_state_payload_error_frames=sum(item["bit_errors"] > 0 and item["state"] != 0 for item in eligible),
                    zero_state_payload_error_frames=sum(item["bit_errors"] > 0 and item["state"] == 0 for item in eligible),
                    bit_errors=errors, pixel_errors=sum(item["pixel_errors"] for item in eligible),
                    compared_bits=bits, conditional_ber=errors / bits if bits else None,
                    conditional_ber_scope=BER_SCOPE,
                    # 档1: layered indicators that are not locked to zero by
                    # the strict header gate.
                    packet_error_rate=per,
                    local_frames=local,
                    local_fcs_fail_frames=fcs_fail,
                    local_fcs_fail_rate=fcs_fail / local if local else None,
                    header_corrupt_rate=self.receiver.header_corrupt / local if local else None,
                    phy_metadata_samples=self.receiver.phy_metadata_samples,
                    phy_rate_histogram=dict(self.receiver.phy_rate_histogram),
                    phy_rate_mbps_histogram=dict(self.receiver.phy_rate_mbps_histogram),
                    phy_bb_format_histogram=dict(self.receiver.phy_bb_format_histogram),
                    phy_second_histogram=dict(self.receiver.phy_second_histogram),
                    phy_noise_floor_mean_dbm=(self.receiver.phy_noise_floor_total /
                                              self.receiver.phy_metadata_samples
                                              if self.receiver.phy_metadata_samples else None),
                    foreign_weak_candidates=self.receiver.foreign_weak_candidates,
                    # 档3: raw FCS-failed payload diagnostics.
                    raw_fcs_error_blocks=len(self.receiver.raw_frames),
                    raw_fcs_error_weak_hint_blocks=sum(
                        1 for hint in self.receiver.raw_hints.values() if hint & 8),
                    dimension_mismatch=self.dimension_mismatch, duplicates=self.receiver.duplicates,
                    header_corrupt=self.receiver.header_corrupt, truncated=self.receiver.truncated,
                    inconsistent=self.receiver.inconsistent, unidentified=self.receiver.unidentified,
                    state_histogram=dict(self.receiver.state_histogram))

    def error_mask(self):
        if self.receiver.size != self.size:
            return None
        mask = Image.new("RGB", self.size, (100, 100, 100))
        pixels = bytearray(mask.tobytes())
        for sequence, frame in self.receiver.frames.items():
            offset = sequence * CHUNK
            for i in range(0, frame["length"], 3):
                damaged = self.source[offset+i:offset+i+3] != frame["payload"][i:i+3]
                pixels[offset+i:offset+i+3] = bytes((255, 40, 40) if damaged else (30, 170, 80))
        return Image.frombytes("RGB", self.size, bytes(pixels))
