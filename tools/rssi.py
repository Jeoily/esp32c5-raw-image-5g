"""RSSI of identifiable first-received image blocks, timed at host processing."""
from collections import deque


class RssiTracker:
    def __init__(self):
        self.samples = deque()
        self.last_time = None
        self.last = None
        self.count = 0
        self.total = 0
        self.minimum = None
        self.maximum = None

    def add(self, value, now):
        self.last = value
        self.last_time = now
        self.samples.append((now, value))
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        self.prune(now)

    def prune(self, now):
        while self.samples and now - self.samples[0][0] > 1:
            self.samples.popleft()

    def label(self, now, active):
        self.prune(now)
        if self.last_time is None:
            return "Bob RSSI：— · 尚无可识别图像块"
        age = max(0, now - self.last_time)
        if not active:
            return f"Bob RSSI：最后 {self.last} dBm · 非实时值（发送与收尾已结束）"
        if age > 1:
            return f"Bob RSSI：— · {age:.1f}秒未收到可识别新块；上次 {self.last} dBm"
        mean = sum(value for _, value in self.samples) / len(self.samples)
        return f"Bob RSSI：最新 {self.last} dBm · 近1秒均值 {mean:.1f} dBm · 距更新 {age:.1f}秒"

    def summary(self):
        return dict(rssi_samples=self.count, rssi_last_dbm=self.last,
                    rssi_mean_dbm=self.total/self.count if self.count else None,
                    rssi_min_dbm=self.minimum, rssi_max_dbm=self.maximum)
