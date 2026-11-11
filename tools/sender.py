"""Serial flow control only. No receiver ACKs and no RF retransmissions."""
import queue
import time

from protocol import CHUNK, command_record


def transmit(run, size, pixels, source_mac, interval_ms, power, ports, acknowledgments, stop, notify,
             configure_peer=True):
    def write(role, body):
        ports[role].write(command_record(body))

    def request(role, body, expected):
        if stop.is_set():
            raise InterruptedError("用户停止")
        write(role, body)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if stop.is_set():
                raise InterruptedError("用户停止")
            try:
                reply_role, fields = acknowledgments.get(timeout=0.1)
            except queue.Empty:
                continue
            if fields[0] == "ERROR":
                raise RuntimeError(reply_role + " " + " ".join(fields))
            if role == reply_role and fields[:len(expected)] == expected:
                return fields
        raise TimeoutError(f"等待{role}的{expected[0]}超时；本块不会自动重发")

    complete = False
    reason = "发送结束"
    try:
        if configure_peer:
            request("Bob", "PEER " + source_mac, ["PEER_OK"])
        request("Alice", "STOP", ["STOPPED"])
        request("Alice", f"BEGIN {run} {size[0]} {size[1]} {power}", ["START", str(run)])
        count = (len(pixels) + CHUNK - 1) // CHUNK
        # Four commands fit within the firmware's 4096-byte UART RX buffer.
        # Pipeline only in fast mode; paced mode keeps the previous behavior.
        window = 4 if interval_ms == 0 else 1
        pending = {}
        sequence = 0
        acknowledged = 0
        next_send = 0
        while acknowledged < count:
            if stop.is_set():
                raise InterruptedError("用户停止")
            now = time.monotonic()
            while sequence < count and len(pending) < window and now >= next_send:
                if stop.is_set():
                    raise InterruptedError("用户停止")
                block = pixels[sequence*CHUNK:(sequence+1)*CHUNK]
                write("Alice", f"BLOCK {run} {sequence} {block.hex()}")
                pending[sequence] = time.monotonic() + 5
                sequence += 1
                next_send = time.monotonic() + interval_ms / 1000
                now = time.monotonic()
            if pending and now >= min(pending.values()):
                raise TimeoutError("等待Alice发送确认超时；不重发")
            if not pending:
                stop.wait(min(0.01, max(0, next_send-now)))
                continue
            try:
                reply_role, response = acknowledgments.get(timeout=0.01)
            except queue.Empty:
                continue
            if response[0] == "ERROR":
                raise RuntimeError(reply_role + " " + " ".join(response))
            if reply_role != "Alice" or response[:2] != ["TX", str(run)]:
                continue
            if len(response) != 4 or int(response[2]) not in pending:
                raise RuntimeError("Alice发送确认重复或不属于当前窗口")
            del pending[int(response[2])]
            if int(response[3]) != 0:
                raise RuntimeError("Alice发送API失败，停止本轮，不补发；已排队块可能仍会发出")
            acknowledged += 1
        request("Alice", f"END {run}", ["DONE", str(run), str(count)])
        complete = True
    except Exception as exc:
        reason = str(exc)
    finally:
        try:
            write("Alice", "STOP")
        except Exception as exc:
            complete = False
            reason += "；停止命令未送达：" + str(exc)
        notify(complete, reason)
