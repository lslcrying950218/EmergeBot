#!/usr/bin/env python3
"""
Lightweight log watcher for DimOS agent-driven monitoring.

Tails the dimos JSONL log file and continuously updates a state file with
the latest robot status. The agent reads this state file to make decisions.

This script does NOT make any decisions or call any MCP commands.
It only processes logs and writes state.

Usage:
    python3 -u log_watcher.py [--log /path/to/main.jsonl] [--state /tmp/dimos_state.json] [--interval 3]

Output state file format:
{
    "state": "EXPLORING" | "FOLLOWING" | "LOST" | "IDLE",
    "last_event": "human readable summary",
    "last_event_time": "HH:MM:SS",
    "cycle": 1,
    "log_ts": "ISO timestamp of last log entry",
    "age_seconds": 5.2,          // seconds since last log entry
    "summary": {                  // counts since last state change
        "no_path_count": 0,
        "frontier_goals": 3,
        "lost_count": 0,
        "found_count": 1
    }
}
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

DEFAULT_INTERVAL = 3


def get_log_path():
    """Auto-discover log path from dimos status."""
    result = subprocess.run(
        ["bash", "-c",
         "cd /home/millozhang/Documents/WIL/reference/dimos && "
         "source .venv/bin/activate && dimos status"],
        capture_output=True, text=True, timeout=10,
    )
    for line in result.stdout.splitlines():
        if "Log:" in line:
            log_dir = line.split("Log:")[-1].strip()
            return os.path.join(log_dir, "main.jsonl")
    return None


def parse_event(msg):
    """Classify a log event into categories."""
    msg_lower = msg.lower()

    # Following / found
    for kw in ("edgetam initialized", "found the person", "warmup locked",
               "starting to follow"):
        if kw in msg_lower:
            return "FOUND"

    # Lost
    for kw in ("lost track of the person", "person follow stopped"):
        if kw in msg_lower:
            return "LOST"

    # Exploration stopped
    for kw in ("stopped autonomous frontier exploration", "no information gain"):
        if kw in msg_lower:
            return "EXPLORATION_STOPPED"

    # No path
    if "no path found" in msg_lower:
        return "NO_PATH"

    return None


def tail_new(log_path, last_pos):
    """Read new lines from log file since last_pos."""
    events = []
    latest_ts = None
    try:
        with open(log_path, 'r') as f:
            f.seek(last_pos)
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    e = json.loads(line)
                    msg = str(e.get('event', e.get('msg', e.get('message', ''))))
                    ts = e.get('timestamp', '')
                    events.append((msg, ts))
                    if ts:
                        latest_ts = ts
                except (json.JSONDecodeError, Exception):
                    pass
            new_pos = f.tell()
    except Exception:
        return events, last_pos, latest_ts
    return events, new_pos, latest_ts


def compute_state(state, events, no_path_count):
    """Process events and return new state + counts."""
    found = False
    lost = False
    exploration_stopped = False
    new_no_path = 0
    last_event = ""

    for msg, ts in events:
        etype = parse_event(msg)
        if etype == "FOUND":
            found = True
            last_event = msg[:120]
        elif etype == "LOST":
            lost = True
            last_event = msg[:120]
        elif etype == "EXPLORATION_STOPPED":
            exploration_stopped = True
            last_event = msg[:120]
        elif etype == "NO_PATH":
            new_no_path += 1
            last_event = msg[:120]
        elif "frontier" in msg.lower() or "goal" in msg.lower():
            last_event = msg[:120]

    no_path_count += new_no_path

    # State transitions
    if state == "EXPLORING":
        if found:
            state = "FOLLOWING"
            no_path_count = 0
        elif exploration_stopped:
            state = "EXPLORATION_STOPPED"
    elif state == "FOLLOWING":
        if lost:
            state = "LOST"
    elif state == "LOST":
        pass  # agent handles
    elif state == "EXPLORATION_STOPPED":
        if found:
            state = "FOLLOWING"

    return state, no_path_count, last_event or "(no notable event)"


def main():
    parser = argparse.ArgumentParser(description="Lightweight DimOS log watcher")
    parser.add_argument("--log", default=None, help="Path to main.jsonl (auto-detected if omitted)")
    parser.add_argument("--state", default="/tmp/dimos_state.json", help="State file path")
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL,
                        help=f"Poll interval in seconds (default: {DEFAULT_INTERVAL})")
    args = parser.parse_args()

    log_path = args.log
    state_file = args.state

    # Auto-detect log path
    if not log_path:
        log_path = get_log_path()
        if not log_path:
            print("[watcher] ERROR: Could not detect log path. Is dimos running?", flush=True)
            sys.exit(1)

    if not os.path.exists(log_path):
        print(f"[watcher] ERROR: Log file not found: {log_path}", flush=True)
        sys.exit(1)

    # Start from end of file
    last_pos = os.path.getsize(log_path)
    state = "EXPLORING"
    cycle = 0
    no_path_count = 0

    print(f"[watcher] Started. Log: {log_path}", flush=True)
    print(f"[watcher] State file: {state_file}", flush=True)
    print(f"[watcher] Interval: {args.interval}s", flush=True)

    try:
        while True:
            time.sleep(args.interval)

            # Re-detect log path if needed (survives dimos restart)
            if not os.path.exists(log_path):
                new_path = get_log_path()
                if new_path and new_path != log_path:
                    log_path = new_path
                    last_pos = 0
                    print(f"[watcher] Log path changed: {log_path}", flush=True)
                continue

            events, last_pos, latest_ts = tail_new(log_path, last_pos)
            state, no_path_count, last_event = compute_state(
                state, events, no_path_count)

            # Track cycles
            if state == "FOLLOWING" and events:
                for msg, _ in events:
                    if "found the person" in msg.lower() or "edgetam initialized" in msg.lower():
                        cycle += 1

            # Compute age of latest log entry
            age_seconds = -1
            if latest_ts:
                try:
                    t = datetime.fromisoformat(latest_ts.replace('Z', '+00:00'))
                    age_seconds = (datetime.now(timezone.utc) - t).total_seconds()
                except Exception:
                    pass

            # Write state file
            state_data = {
                "state": state,
                "last_event": last_event,
                "last_event_time": datetime.now().strftime("%H:%M:%S"),
                "cycle": cycle,
                "log_ts": latest_ts or "",
                "age_seconds": round(age_seconds, 1),
                "log_path": log_path,
                "summary": {
                    "no_path_count": no_path_count,
                }
            }
            tmp_file = state_file + ".tmp"
            with open(tmp_file, 'w') as f:
                json.dump(state_data, f, indent=2)
            os.replace(tmp_file, state_file)

    except KeyboardInterrupt:
        print("\n[watcher] Stopped.", flush=True)
        if os.path.exists(state_file):
            os.remove(state_file)


if __name__ == "__main__":
    main()
