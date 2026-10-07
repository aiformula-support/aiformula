#!/usr/bin/env python3
"""Sample `tegrastats` for a fixed duration and print averaged CPU/GPU/RAM/power figures.

Used to compare the load of the ZED wrapper alone, the detector alone and the full pipeline,
so that a change can be attributed to the CPU side or the GPU side.

    ./summarize_tegrastats.py --duration 20 --label "zed only"
"""

import argparse
import re
import subprocess
import sys
import time

CPU_PATTERN = re.compile(r"CPU \[([^\]]+)\]")
CORE_PATTERN = re.compile(r"(\d+)%@(\d+)")
GPU_PATTERN = re.compile(r"GR3D_FREQ (\d+)%")
RAM_PATTERN = re.compile(r"RAM (\d+)/(\d+)MB")
POWER_PATTERN = re.compile(r"(VDD_GPU_SOC|VDD_CPU_CV|VIN_SYS_5V0) (\d+)mW")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=20.0, help="sampling duration in seconds")
    parser.add_argument("--interval", type=int, default=500, help="tegrastats interval in milliseconds")
    parser.add_argument("--label", default="", help="label printed with the summary")
    return parser.parse_args()


def collect(duration: float, interval_ms: int) -> list:
    process = subprocess.Popen(
        ["tegrastats", "--interval", str(interval_ms)],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    lines = []
    try:
        start = time.perf_counter()
        while time.perf_counter() - start < duration:
            line = process.stdout.readline()
            if not line:
                break
            lines.append(line)
    finally:
        process.terminate()
        process.wait()
    return lines


def summarize(lines: list, label: str) -> None:
    if not lines:
        print("tegrastats produced no output; is it installed and runnable?", file=sys.stderr)
        sys.exit(1)

    per_core_loads = []
    gpu_loads = []
    ram_used = []
    power = {}
    for line in lines:
        cpu_match = CPU_PATTERN.search(line)
        if cpu_match:
            per_core_loads.append([int(load) for load, _ in CORE_PATTERN.findall(cpu_match.group(1))])
        gpu_match = GPU_PATTERN.search(line)
        if gpu_match:
            gpu_loads.append(int(gpu_match.group(1)))
        ram_match = RAM_PATTERN.search(line)
        if ram_match:
            ram_used.append(int(ram_match.group(1)))
        for rail, milliwatts in POWER_PATTERN.findall(line):
            power.setdefault(rail, []).append(int(milliwatts))

    def mean(values):
        return sum(values) / len(values) if values else float("nan")

    print(f"=== tegrastats summary {f'[{label}] ' if label else ''}({len(lines)} samples) ===")
    if per_core_loads:
        core_count = min(len(loads) for loads in per_core_loads)
        core_means = [mean([loads[core] for loads in per_core_loads]) for core in range(core_count)]
        print(f"CPU mean of all cores : {mean(core_means):5.1f} %")
        print(f"CPU busiest core      : {max(core_means):5.1f} % (core {core_means.index(max(core_means))})")
        print("CPU per core          : " + " ".join(f"{value:.0f}" for value in core_means))
    print(f"GPU (GR3D_FREQ)       : {mean(gpu_loads):5.1f} % mean, {max(gpu_loads) if gpu_loads else 0:3d} % peak")
    print(f"RAM used              : {mean(ram_used):7.0f} MB")
    for rail, samples in power.items():
        print(f"{rail:<22}: {mean(samples):7.0f} mW")


def main() -> None:
    args = parse_args()
    summarize(collect(args.duration, args.interval), args.label)


if __name__ == "__main__":
    main()
