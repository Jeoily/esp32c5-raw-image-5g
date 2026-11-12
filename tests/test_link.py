import sys
from pathlib import Path
import unittest
import queue
import threading
import tempfile
import time
import json
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from PIL import Image
from protocol import *
from sender import transmit
from rssi import RssiTracker

MAC = '102030405060'

def rx(raw, state=0):
    return f'RX 100 2 {state} -60 40 {len(raw)} {len(raw)+4} {len(raw)} {raw.hex()}'.split()

class LinkTests(unittest.TestCase):
    def test_rssi_fresh_stale_and_reset(self):
        tracker = RssiTracker()
        self.assertIn('尚无', tracker.label(0, True))
        tracker.add(-39, 1)
        tracker.add(-77, 1.5)
        self.assertIn('最新 -77 dBm', tracker.label(1.5, True))
        self.assertIn('均值 -58.0', tracker.label(1.5, True))
        self.assertIn('均值 -77.0', tracker.label(2.1, True))
        self.assertIn('未收到', tracker.label(3, True))
        self.assertIn('非实时值', tracker.label(3, False))
        self.assertEqual(tracker.summary()['rssi_mean_dbm'], -58)
        self.assertEqual(tracker.summary()['rssi_samples'], 2)
        self.assertIsNone(RssiTracker().summary()['rssi_last_dbm'])

    def test_reordered_exact_and_partial(self):
        for size in ((1, 1), (81, 3), (256, 192), (640, 480)):
            data = bytes(i % 251 for i in range(size[0]*size[1]*3))
            receiver = ImageReceiver(7, MAC)
            for seq in reversed(range((len(data)+239)//240)):
                receiver.feed(rx(encode_frame(7, seq, *size, MAC, data[seq*240:(seq+1)*240])))
            self.assertEqual(receiver.image().tobytes(), data)
            self.assertEqual(set(receiver.missing_mask().tobytes()), {0})

    def test_error_preserved_duplicate_never_repairs(self):
        source = Image.new('RGB', (160, 1), (1, 2, 3))
        run = ImageRun(7, MAC, source)
        run.submitted = {0, 1}
        payload = bytearray(source.tobytes()[:240]); payload[3] ^= 7
        run.receive(rx(encode_frame(7, 0, 160, 1, MAC, payload), 65))
        run.receive(rx(encode_frame(7, 0, 160, 1, MAC, source.tobytes()[:240])))
        self.assertEqual(run.receiver.pixels[:240], payload)
        self.assertEqual(run.receiver.pixels[240:], bytes(MISSING_COLOR)*80)
        self.assertEqual(run.summary()['bit_errors'], 3)
        self.assertEqual(run.summary()['pixel_errors'], 1)
        self.assertEqual(run.summary()['missing_blocks'], 1)
        self.assertEqual(run.error_mask().getpixel((1, 0)), (255, 40, 40))
        self.assertEqual(source.getpixel((1, 0)), (1, 2, 3))

    def test_reject_untrusted_headers(self):
        receiver = ImageReceiver(7, MAC)
        frame = encode_frame(7, 0, 1, 1, MAC, b'abc')
        corrupt = bytearray(frame); corrupt[40] ^= 1
        for raw in (corrupt, frame[:-1], encode_frame(8, 0, 1, 1, MAC, b'abc'),
                    encode_frame(7, 0, 1, 1, '112233445566', b'abc'),
                    encode_frame(7, 0, 641, 1, MAC, b'abc'),
                    encode_frame(7, 2, 1, 1, MAC, b'abc')):
            self.assertIsNone(receiver.feed(rx(raw)))
        self.assertIsNone(receiver.image())

    def test_serial_crc(self):
        text = command_record('BLOCK 7 0 abc').decode().strip()
        self.assertEqual(parse_record(text), ['BLOCK', '7', '0', 'abc'])
        with self.assertRaises(ValueError):
            parse_record(text.replace('abc', 'abd'))

    def test_sender_one_submission_and_failure(self):
        for failure in (False, True):
            acks = queue.Queue(); writes = []; result = []
            class Port:
                def __init__(self, role): self.role = role
                def write(self, data):
                    fields = parse_record(data.decode().strip()); writes.append(fields)
                    replies = {'PEER': ['PEER_OK'], 'STOP': ['STOPPED'],
                               'BEGIN': ['START', '7'], 'END': ['DONE', '7', '2']}
                    reply = (['TX', '7', fields[2], '1' if failure else '0']
                             if fields[0] == 'BLOCK' else replies[fields[0]])
                    acks.put((self.role, reply))
            transmit(7, (160, 1), bytes(480), MAC, 0, 32,
                     {r: Port(r) for r in ('Alice', 'Bob')}, acks,
                     threading.Event(), lambda *args: result.append(args))
            self.assertEqual(result[0][0], not failure)
            blocks = [f[2] for f in writes if f[0] == 'BLOCK']
            self.assertEqual(blocks, ['0', '1'])

    def test_pipeline_bounded_and_stop(self):
        acks = queue.Queue(); blocks = []; result = []; stop = threading.Event()
        class Port:
            def __init__(self, role): self.role = role
            def write(self, data):
                f = parse_record(data.decode().strip())
                if f[0] == 'BLOCK':
                    blocks.append(int(f[2]))
                    return
                reply = {'PEER': ['PEER_OK'], 'STOP': ['STOPPED'], 'BEGIN': ['START', '7']}[f[0]]
                acks.put((self.role, reply))
        worker = threading.Thread(target=transmit, args=(7, (640, 1), bytes(1920), MAC, 0, 32,
            {r: Port(r) for r in ('Alice', 'Bob')}, acks, stop, lambda *args: result.append(args)))
        worker.start()
        try:
            deadline = time.monotonic()+2
            while len(blocks)<4 and time.monotonic()<deadline: time.sleep(.005)
            self.assertEqual(blocks, [0, 1, 2, 3])
            time.sleep(.05)
            self.assertEqual(len(blocks), 4)
            acks.put(('Alice', ['TX', '7', '0', '0']))
            deadline = time.monotonic()+2
            while len(blocks)<5 and time.monotonic()<deadline: time.sleep(.005)
            self.assertEqual(blocks, [0, 1, 2, 3, 4])
        finally:
            stop.set(); worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertFalse(result[0][0])

    def test_gui_transfer_export(self):
        self.exercise_gui(False)

    def test_background_save_failure_stops_loop(self):
        import app
        with tempfile.TemporaryDirectory() as folder, patch.object(app, 'ROOT', Path(folder)):
            window = app.ImageApp(); window.withdraw()
            try:
                window.loop_enabled = True
                run = ImageRun(7, MAC, Image.new('RGB', (1, 1)))
                with patch.object(app, 'save_round', side_effect=OSError('test disk failure')):
                    window.queue_save(run, Path(folder), {})
                    deadline = time.monotonic()+2
                    while window.saves and time.monotonic()<deadline:
                        window.update(); time.sleep(.01)
                self.assertFalse(window.saves)
                self.assertFalse(window.loop_enabled)
                self.assertIn('test disk failure', window.logbox.get('1.0', 'end'))
            finally:
                window.close()

    def test_gui_loop_two_rounds_and_stop(self):
        self.exercise_gui(True)

    def exercise_gui(self, looping):
        import app
        with tempfile.TemporaryDirectory() as folder, patch.object(app, 'ROOT', Path(folder)):
            window = app.ImageApp(); window.withdraw()
            try:
                window.source_image = Image.new('RGB', (240, 1), (10, 20, 30))
                window.selected_path = Path('offline-test.png')
                window.interval.set('0')
                begins = []
                delayed = []
                carried_rssi = []
                for role in ('Alice', 'Bob'):
                    window.info[role] = [role, '40', MAC, 'v6.0', FIRMWARE]
                class Port:
                    def __init__(self, role): self.role = role
                    def close(self): pass
                    def write(self, data):
                        f = parse_record(data.decode().strip())
                        reply = {'PEER': 'PEER_OK', 'STOP': 'STOPPED 0 0'}.get(f[0])
                        if f[0] == 'BEGIN':
                            begins.append(time.monotonic())
                            if looping and len(begins) == 2:
                                carried_rssi.append(window.live_rssi.count)
                                for line in delayed:
                                    window.events.put((window.generation, 'Bob', line))
                            reply = 'START ' + ' '.join(f[1:])
                        if f[0] == 'END': reply = f'DONE {f[1]} 3'
                        if f[0] == 'BLOCK':
                            reply = f'TX {f[1]} {f[2]} 0'
                            seq = int(f[2]); payload = bytearray.fromhex(f[3])
                            if seq == 1: payload[0] ^= 1
                            if seq != 2:
                                raw = encode_frame(int(f[1]), seq, 240, 1, MAC, payload)
                                line = command_record(' '.join(rx(raw, 65 if seq == 1 else 0))).decode().strip()
                                window.events.put((window.generation, 'Bob', line))
                            elif looping and len(begins) == 1:
                                raw = encode_frame(int(f[1]), seq, 240, 1, MAC, payload)
                                delayed.append(command_record(' '.join(rx(raw))).decode().strip())
                        window.events.put((window.generation, self.role, command_record(reply).decode().strip()))
                window.ports = {r: Port(r) for r in ('Alice', 'Bob')}
                window.start_loop() if looping else window.start()
                deadline = time.monotonic()+15
                stopped = False
                while (window.active or window.loop_enabled or window.draining or window.saves) and time.monotonic() < deadline:
                    window.update(); time.sleep(.01)
                    if looping and window.loop_round == 2 and window.run.sender_complete and not stopped:
                        window.stop(); stopped = True
                self.assertFalse(window.active)
                self.assertFalse(window.loop_enabled)
                if looping:
                    self.assertTrue(stopped)
                    self.assertEqual(window.loop_round, 2)
                    sessions = list((Path(folder)/'results').glob('*/summary.json'))
                    self.assertEqual(len(sessions), 2)
                    runs = [json.loads(p.read_text(encoding='utf-8'))['run'] for p in sessions]
                    self.assertEqual(len(set(runs)), 2)
                    self.assertIsNone(window.loop_after)
                    self.assertLess(begins[1]-begins[0], 1.0)
                    self.assertEqual(carried_rssi, [2])
                    first = min(sessions, key=lambda p: p.parent.name)
                    first_result = json.loads(first.read_text(encoding='utf-8'))
                    self.assertEqual(first_result['unique_received'], 3)
                    self.assertEqual(first_result['missing_blocks'], 0)
                    self.assertEqual(first_result['rssi_samples'], 3)
                    with Image.open(first.parent/'received.png') as received:
                        self.assertEqual(received.getpixel((160, 0)), (10, 20, 30))
                result = json.loads((window.session/'summary.json').read_text(encoding='utf-8'))
                self.assertTrue(result['sender_complete'])
                self.assertEqual(result['bit_errors'], 1)
                self.assertEqual(result['missing_blocks'], 1)
                self.assertEqual(result['rssi_samples'], 2)
                self.assertEqual(result['rssi_mean_dbm'], -60)
                self.assertIn('非实时值', window.rssi_text.get())
                self.assertGreaterEqual(result['tx_ack_span_seconds'], 0)
                if result['tx_ack_span_seconds'] > 0:
                    self.assertGreater(result['approx_payload_bps'], 0)
                else:
                    self.assertIsNone(result['approx_payload_bps'])
                with Image.open(window.session/'received.png') as received:
                    self.assertEqual(received.getpixel((80, 0)), (11, 20, 30))
                    self.assertEqual(received.getpixel((160, 0)), MISSING_COLOR)
            finally:
                window.close()

def rx_new(raw, state=0, rate=0xB, bb_format=0, second=0,
           noise_floor=-90, he_siga1=0, local_hint=1):
    """16-field record produced by the updated firmware."""
    return rx(raw, state) + [str(rate), str(bb_format), str(second),
                             str(noise_floor), str(he_siga1), str(local_hint)]


class LayeredStatisticsTests(unittest.TestCase):
    def test_old_record_still_accepted(self):
        source = Image.new('RGB', (160, 1), (1, 2, 3))
        run = ImageRun(7, MAC, source)
        run.submitted = {0}
        run.receive(rx(encode_frame(7, 0, 160, 1, MAC, source.tobytes()[:240])))
        summary = run.summary()
        self.assertEqual(summary['phy_metadata_samples'], 0)
        self.assertEqual(summary['local_frames'], 1)
        self.assertEqual(summary['local_fcs_fail_frames'], 0)
        self.assertEqual(summary['packet_error_rate'], 0)
        self.assertEqual(summary['raw_fcs_error_blocks'], 0)
        self.assertIn('丢包', summary['conditional_ber_scope'])
        self.assertIsNone(run.receiver.raw_error_image())

    def test_new_record_phy_histograms_and_per(self):
        source = Image.new('RGB', (240, 1), (4, 5, 6))
        run = ImageRun(7, MAC, source)
        run.submitted = {0, 1, 2}
        run.receive(rx_new(encode_frame(7, 0, 240, 1, MAC, source.tobytes()[:240]),
                           state=0, rate=0xB, noise_floor=-92))
        run.receive(rx_new(encode_frame(7, 1, 240, 1, MAC, source.tobytes()[240:480]),
                           state=65, rate=0xB, noise_floor=-88, local_hint=9))
        summary = run.summary()
        self.assertEqual(summary['phy_metadata_samples'], 2)
        self.assertEqual(summary['phy_rate_mbps_histogram'], {'6': 2})
        self.assertEqual(summary['phy_second_histogram'], {'0': 2})
        self.assertEqual(summary['phy_noise_floor_mean_dbm'], -90)
        self.assertEqual(summary['local_frames'], 2)
        self.assertEqual(summary['local_fcs_fail_frames'], 1)
        self.assertEqual(summary['local_fcs_fail_rate'], 0.5)
        self.assertEqual(summary['foreign_weak_candidates'], 1)
        self.assertAlmostEqual(summary['packet_error_rate'], 1 / 3)
        self.assertEqual(summary['raw_fcs_error_blocks'], 1)

    def test_raw_error_image_uses_damaged_header_payload(self):
        source = Image.new('RGB', (160, 1), (1, 2, 3))
        run = ImageRun(7, MAC, source)
        run.submitted = {0}
        payload = bytes((200, 100, 50) * 80)
        frame = bytearray(encode_frame(7, 0, 160, 1, MAC, payload))
        frame[52] ^= 1  # corrupt the header CRC only
        run.receive(rx_new(bytes(frame), state=65, local_hint=9))
        # The strict gate must still reject it for the primary image.
        self.assertIsNone(run.receiver.image())
        self.assertEqual(run.summary()['header_corrupt'], 1)
        # ... but raw_error.png shows the original, unverified bytes.
        raw_image = run.receiver.raw_error_image()
        self.assertIsNotNone(raw_image)
        self.assertEqual(raw_image.getpixel((0, 0)), (200, 100, 50))
        self.assertEqual(run.receiver.raw_error_mask().getpixel((79, 0)), 255)
        self.assertEqual(run.summary()['raw_fcs_error_blocks'], 1)
        self.assertEqual(run.summary()['local_fcs_fail_frames'], 1)

    def test_raw_error_never_repairs_primary_image(self):
        source = Image.new('RGB', (240, 1), (9, 9, 9))
        run = ImageRun(7, MAC, source)
        run.submitted = {0, 1}
        damaged = bytearray(encode_frame(7, 0, 240, 1, MAC, bytes((7, 7, 7) * 80)))
        damaged[52] ^= 1
        run.receive(rx_new(bytes(damaged), state=65, local_hint=9))
        run.receive(rx_new(encode_frame(7, 1, 240, 1, MAC, source.tobytes()[240:480]),
                           state=0, local_hint=1))
        primary = run.receiver.image()
        self.assertEqual(primary.getpixel((0, 0)), MISSING_COLOR)
        self.assertEqual(primary.getpixel((80, 0)), (9, 9, 9))
        self.assertEqual(run.receiver.raw_error_image().getpixel((0, 0)), (7, 7, 7))

    def test_gui_routes_new_format_to_draining_round(self):
        # Call the unbound handler with a lightweight stand-in instead of
        # creating a second Tk root, which would disturb the GUI test suite.
        import io
        import types
        import app
        old_run = ImageRun(7, MAC, Image.new('RGB', (160, 1), (1, 2, 3)))
        old_run.submitted = {0}
        stand_in = types.SimpleNamespace(
            connection_log=None, capture_log=None, live_rssi=RssiTracker(),
            draining={7: dict(run=old_run, log=io.StringIO(), rssi=RssiTracker(), result={})})
        line = command_record(' '.join(rx_new(
            encode_frame(7, 0, 160, 1, MAC, bytes((3, 4, 5) * 80)),
            state=0, rate=0xB))).decode().strip()
        app.ImageApp.handle(stand_in, 'Bob', line)
        self.assertEqual(old_run.receiver.frames[0]['payload'][:3], b'\x03\x04\x05')
        self.assertEqual(old_run.summary()['phy_rate_mbps_histogram'], {'6': 1})
        self.assertIn('!RX', stand_in.draining[7]['log'].getvalue())


if __name__ == '__main__':
    unittest.main(verbosity=2)
