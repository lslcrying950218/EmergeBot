#!/usr/bin/env python3
"""
Log-based auto-recovery loop: explore -> lookout (then follow) -> restart on loss.

Monitors dimos JSONL logs for state transitions and automatically restarts the
explore-detect-follow cycle when:
  - Target is lost ("Person follow stopped" / "lost track of the person")
  - Exploration stops unexpectedly ("Stopped autonomous frontier exploration")
  - Exploration stalls ("No information gain" or "No path found" repeated)
  - No progress for a configurable timeout

Usage:
    python log_monitor_loop.py --target "person holding a green cup" \
        --desc "person holding a green cup" "human with green cup" "green cup"

    # With custom check interval and stale timeout:
    python log_monitor_loop.py --target "person" \
        --desc "person" --check 10 --stale 120

The script auto-discovers the current Run ID from `dimos status` so it survives
`dimos restart` without hardcoding paths.

Note: Run with `python3 -u` or redirect to a file to ensure unbuffered output
when running in the background.
"""

import argparse
import json
import os
import subprocess
import sys
import time

CHECK_INTERVAL = 8.0
RESTART_DELAY = 4.0
STALE_EXPLORATION_TIMEOUT = 90.0
NO_PATH_THRESHOLD = 8  # consecutive "No path found" before forced restart


def dimos_status() -> dict:
    """Parse `dimos status` output into a dict."""
    result = subprocess.run(
        ["bash", "-c",
         "/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos status"],
        capture_output=True, text=True,
    )
    out = result.stdout
    data = {}
    for line in out.splitlines():
        if "Run ID:" in line:
            data["run_id"] = line.split("Run ID:")[-1].strip()
        elif "Blueprint:" in line:
            data["blueprint"] = line.split("Blueprint:")[-1].strip()
        elif "Log:" in line:
            data["log"] = line.split("Log:")[-1].strip()
    return data


def get_log_path() -> str:
    status = dimos_status()
    log = status.get("log")
    if log:
        return os.path.join(log, "main.jsonl")
    raise RuntimeError("Could not determine log path from dimos status")


def mcp_call(tool: str, args: dict | None = None) -> str:
    json_args = f" --json-args '{json.dumps(args)}'" if args else ""
    cmd = [
        "bash", "-c",
        f"/home/emergeos/Share_pgx/ZLP/dimos/.venv/bin/dimos mcp call {tool}{json_args}",
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.stdout.strip()


def cleanup():
    """Stop all active skills."""
    mcp_call("end_exploration")
    mcp_call("stop_following")
    mcp_call("stop_looking_out")
    mcp_call("stop_navigation")


def restart_cycle(target: str, descriptions: list[str]):
    """Start a fresh explore + detect cycle."""
    print("[LOOP] >>> Cleaning up and restarting cycle", flush=True)
    cleanup()
    time.sleep(1.5)

    r1 = mcp_call("begin_exploration")
    print(f"[LOOP] begin_exploration -> {r1}", flush=True)

    r2 = mcp_call("look_out_for", {
        "description_of_things": descriptions,
        "then": {"name": "follow_person", "arguments": {"query": target}},
    })
    print(f"[LOOP] look_out_for -> {r2}", flush=True)


def tail_logs(log_path: str, last_pos: int) -> tuple[list[str], int]:
    if not os.path.exists(log_path):
        return [], last_pos
    with open(log_path, "r") as f:
        f.seek(last_pos)
        lines = f.readlines()
        new_pos = f.tell()
    return lines, new_pos


def run_loop(target: str, descriptions: list[str], check_interval: float,
             stale_timeout: float):
    state = "exploring"
    last_event_time = time.monotonic()
    last_log_pos = 0
    no_path_count = 0

    # Initialize log position to end
    log_path = get_log_path()
    if os.path.exists(log_path):
        with open(log_path, "r") as f:
            f.seek(0, 2)
            last_log_pos = f.tell()

    print(f"[LOOP] Monitor started. Target: {target}", flush=True)
    print(f"[LOOP] Log: {log_path}", flush=True)
    restart_cycle(target, descriptions)

    try:
        while True:
            time.sleep(check_interval)

            # Re-resolve log path in case dimos was restarted
            try:
                current_log = get_log_path()
            except RuntimeError:
                print("[LOOP] dimos not running, waiting...", flush=True)
                continue

            if current_log != log_path:
                print(f"[LOOP] Run ID changed! New log: {current_log}", flush=True)
                log_path = current_log
                last_log_pos = 0
                if os.path.exists(log_path):
                    with open(log_path, "r") as f:
                        f.seek(0, 2)
                        last_log_pos = f.tell()

            lines, last_log_pos = tail_logs(log_path, last_log_pos)
            need_restart = False

            for line in lines:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                event = rec.get("event", "")

                # --- Target found / following ---
                if any(k in event for k in ("POLLING_MATCH", "CONTINUATION_TRIGGER",
                                             "Found the person. Starting to follow.",
                                             "EdgeTAM initialized")):
                    if state != "following":
                        print(f"[LOOP] Target found -> follow starting ({event[:80]})",
                              flush=True)
                        state = "following"
                        last_event_time = time.monotonic()
                        no_path_count = 0

                # --- Target lost ---
                if any(k in event for k in ("Person follow stopped", "lost track of the person")):
                    print(f"[LOOP] Follow stopped: {event}", flush=True)
                    if state == "following":
                        need_restart = True
                        break

                # --- Exploration stalled ---
                if "Stopped autonomous frontier exploration" in event:
                    print(f"[LOOP] Exploration stopped unexpectedly", flush=True)
                    if state == "exploring":
                        need_restart = True
                        break

                if "No information gain" in event:
                    print(f"[LOOP] No information gain, exploration exhausted", flush=True)
                    if state == "exploring":
                        need_restart = True
                        break

                if "No path found to the goal" in event:
                    no_path_count += 1
                    if no_path_count > NO_PATH_THRESHOLD and state == "exploring":
                        print(f"[LOOP] Too many no-path failures ({no_path_count})",
                              flush=True)
                        need_restart = True
                        break

            # --- Stale timeout ---
            if not need_restart:
                stale = time.monotonic() - last_event_time > stale_timeout
                if stale and state == "exploring":
                    print("[LOOP] No progress for too long, forcing restart", flush=True)
                    need_restart = True

            if need_restart:
                state = "exploring"
                no_path_count = 0
                time.sleep(RESTART_DELAY)
                restart_cycle(target, descriptions)
                last_event_time = time.monotonic()

    except KeyboardInterrupt:
        print("\n[LOOP] Interrupted. Cleaning up...", flush=True)
        cleanup()
        print("[LOOP] Done.", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Log-based explore-follow auto-recovery loop for DimOS")
    parser.add_argument("--target", default="person",
                        help="Tracking query for follow_person (default: person)")
    parser.add_argument("--desc", nargs="+", default=None,
                        help="Detection descriptions for look_out_for (default: same as --target)")
    parser.add_argument("--check", type=float, default=CHECK_INTERVAL,
                        help=f"Log poll interval in seconds (default: {CHECK_INTERVAL})")
    parser.add_argument("--stale", type=float, default=STALE_EXPLORATION_TIMEOUT,
                        help=f"Stale exploration timeout in seconds (default: {STALE_EXPLORATION_TIMEOUT})")
    args = parser.parse_args()

    target = args.target
    descriptions = args.desc if args.desc else [target]

    print("=" * 50, flush=True)
    print("DimOS Log-Monitor Explore-Follow Loop", flush=True)
    print("=" * 50, flush=True)
    print(f"Target:     {target}", flush=True)
    print(f"Desc:       {descriptions}", flush=True)
    print(f"Check int:  {args.check}s", flush=True)
    print(f"Stale:      {args.stale}s", flush=True)
    print("Press Ctrl+C to stop", flush=True)
    print("=" * 50, flush=True)

    run_loop(target, descriptions, args.check, args.stale)


if __name__ == "__main__":
    main()
