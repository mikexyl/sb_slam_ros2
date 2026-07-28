#!/usr/bin/env python3
"""Sample resource use of the ROS processes involved in a Jetson run."""

import argparse
import csv
import signal
import time
from pathlib import Path

import psutil


COMPONENT_PATTERNS = {
    "kimera_vio": ("mono_vio_node", "stereo_vio_node"),
    "kimera_distributed": ("kimera_distributed_loop_closure",),
    "cbs": ("cbs_ros_node",),
    "zenoh": ("rmw_zenohd",),
    "rosbag": ("ros2 bag play",),
}


def component_for(command: str) -> str | None:
    for component, patterns in COMPONENT_PATTERNS.items():
        if any(pattern in command for pattern in patterns):
            return component
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    known: dict[int, psutil.Process] = {}
    process_path = args.output_dir / "process_metrics.csv"
    system_path = args.output_dir / "system_metrics.csv"
    start = time.monotonic()

    with process_path.open("w", newline="", encoding="utf-8") as process_file, (
        system_path.open("w", newline="", encoding="utf-8")
    ) as system_file:
        process_writer = csv.writer(process_file)
        process_writer.writerow(
            (
                "elapsed_s",
                "component",
                "pid",
                "name",
                "cpu_percent",
                "rss_mib",
                "vms_mib",
                "threads",
                "read_mib",
                "write_mib",
            )
        )
        system_writer = csv.writer(system_file)
        system_writer.writerow(
            (
                "elapsed_s",
                "cpu_percent",
                "memory_used_mib",
                "memory_available_mib",
                "swap_used_mib",
            )
        )
        psutil.cpu_percent(interval=None)

        while running:
            elapsed = time.monotonic() - start
            live_pids: set[int] = set()
            for info in psutil.process_iter(("pid", "name", "cmdline")):
                try:
                    command = " ".join(info.info["cmdline"] or ())
                    component = component_for(command)
                    if component is None:
                        continue
                    process = known.get(info.pid)
                    if process is None:
                        process = info
                        process.cpu_percent(interval=None)
                        known[info.pid] = process
                    live_pids.add(info.pid)
                    memory = process.memory_info()
                    io = process.io_counters()
                    process_writer.writerow(
                        (
                            f"{elapsed:.3f}",
                            component,
                            info.pid,
                            info.info["name"],
                            f"{process.cpu_percent(interval=None):.3f}",
                            f"{memory.rss / 2**20:.3f}",
                            f"{memory.vms / 2**20:.3f}",
                            process.num_threads(),
                            f"{io.read_bytes / 2**20:.3f}",
                            f"{io.write_bytes / 2**20:.3f}",
                        )
                    )
                except (psutil.AccessDenied, psutil.NoSuchProcess):
                    continue

            for pid in set(known) - live_pids:
                known.pop(pid, None)

            memory = psutil.virtual_memory()
            swap = psutil.swap_memory()
            system_writer.writerow(
                (
                    f"{elapsed:.3f}",
                    f"{psutil.cpu_percent(interval=None):.3f}",
                    f"{memory.used / 2**20:.3f}",
                    f"{memory.available / 2**20:.3f}",
                    f"{swap.used / 2**20:.3f}",
                )
            )
            process_file.flush()
            system_file.flush()
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
