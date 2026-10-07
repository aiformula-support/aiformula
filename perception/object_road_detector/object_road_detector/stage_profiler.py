from collections import OrderedDict
from contextlib import contextmanager
import time
from typing import Iterator, List

import torch


class StageProfiler:
    """Accumulates per-stage wall-clock times of the detection pipeline and logs a periodic breakdown.

    GPU work is asynchronous, so a stage that only enqueues CUDA kernels reports almost no time
    while the following synchronization point absorbs it. Enable `cuda_sync` to synchronize at
    every stage boundary and get an honest attribution at the cost of a few percent of throughput.
    """

    def __init__(self, node, enabled: bool, report_interval_sec: float = 5.0, cuda_sync: bool = False) -> None:
        self.logger = node.get_logger()
        self.enabled = enabled
        self.report_interval_sec = report_interval_sec
        self.cuda_sync = cuda_sync and torch.cuda.is_available()
        self.stage_totals = OrderedDict()
        self.frame_count = 0
        self.window_start = time.perf_counter()

    @contextmanager
    def stage(self, name: str) -> Iterator[None]:
        if not self.enabled:
            yield
            return
        self._synchronize()
        start = time.perf_counter()
        try:
            yield
        finally:
            self._synchronize()
            elapsed_ms = (time.perf_counter() - start) * 1e3
            self.stage_totals[name] = self.stage_totals.get(name, 0.0) + elapsed_ms

    def frame_done(self) -> None:
        if not self.enabled:
            return
        self.frame_count += 1
        window_sec = time.perf_counter() - self.window_start
        if window_sec < self.report_interval_sec:
            return
        self.logger.info(self._format_report(window_sec))
        self.stage_totals.clear()
        self.frame_count = 0
        self.window_start = time.perf_counter()

    def _format_report(self, window_sec: float) -> str:
        total_ms = sum(self.stage_totals.values())
        lines: List[str] = [
            f"[profile] {self.frame_count / window_sec:5.2f} FPS over {window_sec:.1f}s, "
            f"{total_ms / self.frame_count:6.2f} ms/frame accounted"
            f"{'' if self.cuda_sync else ' (GPU stages not synchronized)'}"
        ]
        for name, accumulated_ms in self.stage_totals.items():
            per_frame_ms = accumulated_ms / self.frame_count
            share = 100.0 * accumulated_ms / total_ms if total_ms > 0.0 else 0.0
            lines.append(f"    {name:<22} {per_frame_ms:7.2f} ms  {share:5.1f} %")
        return "\n".join(lines)

    def _synchronize(self) -> None:
        if self.cuda_sync:
            torch.cuda.synchronize()
