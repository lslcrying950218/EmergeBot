#!/usr/bin/env python3
"""
Hermes Bridge - Connects Hermes AI agent to EMERGE UI Dashboard.

Data flow:
- Reads conversation history from ~/.hermes/state.db SQLite database
- Sends user messages via Hermes CLI non-interactive mode
- Emits messages to UI via Socket.IO on port 7780

Queue/Worker pattern ensures serial execution of CLI calls.
Approval flow: Dangerous commands detected in CLI stdout are forwarded
to the UI as Socket.IO events. The worker blocks until the user responds.
"""
import sys
# Enable line buffering for stdout so print() statements are flushed immediately
# when running under npm concurrently (pipe mode, not TTY)
sys.stdout.reconfigure(line_buffering=True)

import asyncio
import time
import threading
import os
import re
import sqlite3
import subprocess
import uuid
import queue
from pathlib import Path

import socketio
from starlette.applications import Starlette
from starlette.responses import Response
from starlette.routing import Route
import uvicorn

HERMES_DIR = Path.home() / ".hermes"
DB_PATH = HERMES_DIR / "state.db"
HERMES_CLI = "/home/emergeos/.local/bin/hermes"

# CLI timeout in seconds (increased for tool calls/web queries)
CLI_TIMEOUT_SEC = 180

# Approval wait timeout (how long to wait for user to respond to a dangerous command)
APPROVAL_TIMEOUT_SEC = 290


class HermesBridge:
    def __init__(self, sio: socketio.AsyncServer):
        self._sio = sio
        self._last_id = 0
        self._active_session: str | None = None
        self._new_session_pending = False
        self._running = True
        self._loop: asyncio.AbstractEventLoop | None = None
        # Message queue for serial CLI execution
        self._msg_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self._busy = False

        # Approval flow state
        # Maps approval_id → threading.Event (set when user responds)
        self._approval_events: dict[str, threading.Event] = {}
        # Maps approval_id → choice str ("once"/"session"/"always"/"deny")
        self._approval_choices: dict[str, str] = {}

        # Current running CLI process (for interrupt)
        self._current_process: subprocess.Popen | None = None
        self._process_lock = threading.Lock()

        self._init_db()
        # Start the worker thread
        self._worker_thread = threading.Thread(target=self._cli_worker, daemon=True)
        self._worker_thread.start()

    def _init_db(self):
        """Initialize database tracking with session-based queries."""
        if not DB_PATH.exists():
            print(f"❌ Hermes DB not found at {DB_PATH}")
            return

        try:
            conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
            cursor = conn.cursor()

            # Get the most recent session
            cursor.execute("""
                SELECT session_id FROM messages
                WHERE role IN ('user', 'assistant')
                ORDER BY id DESC LIMIT 1
            """)
            row = cursor.fetchone()
            if row:
                self._active_session = row[0]
                print(f"✅ Active Hermes session: {self._active_session}")

                # Get the last message ID in this session
                cursor.execute("""
                    SELECT MAX(id) FROM messages WHERE session_id = ?
                """, (self._active_session,))
                self._last_id = cursor.fetchone()[0] or 0
                print(f"   Tracking messages from ID: {self._last_id}")

            conn.close()
        except Exception as e:
            print(f"❌ DB Init Error: {e}")

    async def send_history(self, sid: str):
        """Send recent messages from the active session to a newly connected client."""
        if not DB_PATH.exists() or not self._active_session:
            print(f"⚠️ No active session, cannot send history")
            return

        try:
            conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
            cursor = conn.cursor()

            # Get last 15 messages from the active session only
            cursor.execute("""
                SELECT id, role, content, reasoning, reasoning_details, timestamp
                FROM messages
                WHERE session_id = ? AND role IN ('user', 'assistant')
                ORDER BY id DESC LIMIT 15
            """, (self._active_session,))

            rows = cursor.fetchall()
            rows.reverse()  # Oldest first

            for row in rows:
                msg_id, role, content, reasoning, reasoning_details, ts = row
                mapped_role = 'agent' if role == 'assistant' else 'human'

                # Handle tool calls or empty content
                if not content and (reasoning or reasoning_details):
                    display_content = "[系统思考中...]"
                else:
                    display_content = content if content else ""

                # Extract thought from reasoning or reasoning_details
                thought = reasoning
                if reasoning_details and not thought:
                    try:
                        import json
                        details = json.loads(reasoning_details) if isinstance(reasoning_details, str) else reasoning_details
                        thought = details.get('summary', reasoning)
                    except:
                        pass

                payload = {
                    "id": str(msg_id),
                    "role": mapped_role,
                    "content": display_content,
                    "thought": thought if thought else "",
                    "timestamp": time.strftime('%H:%M:%S', time.localtime(ts)) if ts else time.strftime('%H:%M:%S')
                }
                await self._sio.emit("chat_message", payload, room=sid)

            conn.close()
            print(f"📜 Sent {len(rows)} messages from session {self._active_session[:16]}... to client {sid}")
        except Exception as e:
            print(f"❌ Error sending history: {e}")

    def poll_db(self):
        """Poll database for new messages in the active session."""
        while self._running:
            if not DB_PATH.exists():
                time.sleep(1)
                continue

            # If new session is pending and no active session yet, skip polling
            if self._new_session_pending and not self._active_session:
                time.sleep(0.5)
                continue

            # If no active session and not pending, wait
            if not self._active_session:
                time.sleep(1)
                continue

            try:
                conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
                cursor = conn.cursor()

                # Check for session change - but NOT when new_session_pending
                if not self._new_session_pending:
                    cursor.execute("""
                        SELECT session_id FROM messages
                        WHERE role IN ('user', 'assistant')
                        ORDER BY id DESC LIMIT 1
                    """)
                    row = cursor.fetchone()
                    if row and row[0] != self._active_session:
                        self._active_session = row[0]
                        self._last_id = 0
                        print(f"🔄 Switched to new session: {self._active_session}")

                # Get new messages in this session
                cursor.execute("""
                    SELECT id, role, content, reasoning, reasoning_details, timestamp
                    FROM messages
                    WHERE session_id = ? AND id > ? AND role IN ('user', 'assistant')
                    ORDER BY id ASC
                """, (self._active_session, self._last_id))

                for row in cursor.fetchall():
                    msg_id, role, content, reasoning, reasoning_details, ts = row
                    self._last_id = msg_id
                    mapped_role = 'agent' if role == 'assistant' else 'human'

                    if not content and (reasoning or reasoning_details):
                        display_content = "[系统思考中...]"
                    else:
                        display_content = content if content else ""

                    thought = reasoning
                    if reasoning_details and not thought:
                        try:
                            import json
                            details = json.loads(reasoning_details) if isinstance(reasoning_details, str) else reasoning_details
                            thought = details.get('summary', reasoning)
                        except:
                            pass

                    payload = {
                        "id": str(msg_id),
                        "role": mapped_role,
                        "content": display_content,
                        "thought": thought if thought else "",
                        "timestamp": time.strftime('%H:%M:%S', time.localtime(ts)) if ts else time.strftime('%H:%M:%S')
                    }
                    # Emit from the event loop
                    if self._loop and not self._loop.is_closed():
                        asyncio.run_coroutine_threadsafe(
                            self._sio.emit("chat_message", payload),
                            self._loop
                        )
                    print(f"💬 Sent message {msg_id} ({role}) to UI")

                conn.close()
            except Exception as e:
                print(f"⚠️ DB poll error: {e}")

            time.sleep(0.5)

    def _emit_status(self, status: str):
        """Emit hermes_status to all clients."""
        if self._loop and not self._loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self._sio.emit("hermes_status", {"status": status}),
                self._loop
            )

    def _emit_error(self, content: str):
        """Emit error message with proper ID."""
        if self._loop and not self._loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self._sio.emit("chat_message", {
                    "id": f"error-{uuid.uuid4().hex[:8]}",
                    "role": "agent",
                    "content": content,
                    "thought": "",
                    "timestamp": time.strftime('%H:%M:%S')
                }),
                self._loop
            )

    def _emit_approval_request(self, approval_id: str, command: str, description: str):
        """Emit approval_request to all clients."""
        if self._loop and not self._loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self._sio.emit("approval_request", {
                    "id": approval_id,
                    "command": command,
                    "description": description,
                }),
                self._loop
            )

    def _emit_approval_cleared(self):
        """Notify clients that approval is resolved (clear the card)."""
        if self._loop and not self._loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self._sio.emit("approval_cleared", {}),
                self._loop
            )

    def _emit_activity(self, content: str):
        """Emit chat_activity to show real-time Hermes tool progress."""
        if self._loop and not self._loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self._sio.emit("chat_activity", {
                    "id": f"activity-{uuid.uuid4().hex[:8]}",
                    "role": "agent",
                    "kind": "activity",
                    "content": content,
                    "timestamp": time.strftime('%H:%M:%S')
                }),
                self._loop
            )

    def resolve_approval(self, approval_id: str, choice: str):
        """Called from the approval_response Socket.IO handler to unblock the worker."""
        event = self._approval_events.get(approval_id)
        if event:
            self._approval_choices[approval_id] = choice
            event.set()
            print(f"✅ Approval {approval_id[:8]} resolved: {choice}")
        else:
            print(f"⚠️ Unknown approval id: {approval_id[:8]}")

    def queue_message(self, content: str, client_msg_id: str | None = None):
        """Queue a message for sending. Called from send_message handler."""
        msg_id = client_msg_id or f"client-{uuid.uuid4().hex[:8]}"
        self._msg_queue.put((content, msg_id))

    # -------------------------------------------------------------------------
    # Approval prompt detection helpers
    # -------------------------------------------------------------------------

    # The approval prompt from Hermes (approval.py) prints:
    #   Line 1:  ⚠️  DANGEROUS COMMAND: <description>     ← detected by _START_RE
    #   Line 2:       <command text>                       ← collected as command
    #   Line 3:  (blank)
    #   Line 4:       [o]nce  |  [s]ession  |  [a]lways  |  [d]eny   ← detected by _CHOICES_RE
    #   Line 5:       Choice [o/s/a/D]:                    ← detected by _PROMPT_RE (NEW!)
    #
    # After Line 5, Hermes blocks on stdin.readline() waiting for our input.
    # We detect Line 4 OR Line 5 to trigger the approval flow.
    _APPROVAL_START_RE = re.compile(r'DANGEROUS COMMAND[:\s]+(.+)', re.IGNORECASE)
    _APPROVAL_CHOICES_RE = re.compile(r'\[o\]nce\s+\|\s+\[s\]ession', re.IGNORECASE)
    _APPROVAL_PROMPT_RE = re.compile(r'Choice\s+\[o/s/a/D\]', re.IGNORECASE)

    # Map UI choice strings to CLI key presses
    _CHOICE_TO_KEY = {
        "once": "o",
        "session": "s",
        "always": "a",
        "deny": "d",
    }

    # Activity line patterns - lines we want to show as real-time progress
    _ACTIVITY_INCLUDE_RE = re.compile(
        r'(preparing|running|executing|loading|fetching|searching|reading|writing|'
        r'creating|deleting|moving|copying|checking|validating|building|compiling|'
        r'DANGEROUS COMMAND|Allowed once|Allowed for this session|Allowed permanently|'
        r'Denied|✓|❌|⚠️|📚|⚡|🔧|📁|💾|🌐)',
        re.IGNORECASE
    )
    # Lines to always filter out (box drawing, session info, approval prompts)
    _ACTIVITY_EXCLUDE_RE = re.compile(
        r'^(session_id:|╭|╰|─|│|\s*$|\[o\]nce\s*\|\s*\[s\]ession|Choice\s*\[)',
        re.IGNORECASE
    )

    def _activity_from_output(self, line: str) -> str | None:
        """
        Extract activity text from Hermes stdout line.
        Returns None if line should be filtered out.
        """
        stripped = line.strip()
        if not stripped:
            return None
        # Filter out box drawing and approval prompts
        if self._ACTIVITY_EXCLUDE_RE.match(stripped):
            return None
        # Filter out approval choice lines specifically
        if self._APPROVAL_CHOICES_RE.search(stripped) or self._APPROVAL_PROMPT_RE.search(stripped):
            return None
        # Include lines matching activity patterns
        if self._ACTIVITY_INCLUDE_RE.search(stripped):
            return stripped
        return None

    def _run_cli_with_approval(self, cmd: list[str], env: dict) -> tuple[int, str]:
        """
        Run the Hermes CLI via Popen, monitoring stdout for approval prompts.

        When an approval prompt is detected:
          1. Emits approval_request to UI
          2. Blocks until user responds (or timeout)
          3. Writes the choice to the process stdin

        Returns (returncode, combined_output_text).
        """
        # Force unbuffered stdout in the child process so print() calls
        # (including the approval prompt) are immediately readable by us.
        # Without this, Python detects stdout is a pipe and uses full
        # buffering, causing our reader thread to miss output until the
        # buffer fills or the process exits.
        run_env = {**env, 'PYTHONUNBUFFERED': '1'}

        process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # merge stderr into stdout for line-by-line reading
            text=True,
            bufsize=1,  # line-buffered on our side
            env=run_env,
        )

        # Save process reference for interrupt
        with self._process_lock:
            self._current_process = process

        output_lines: list[str] = []
        approval_id: str | None = None
        pending_description: str | None = None
        pending_command_lines: list[str] = []
        waiting_for_choice = False  # True once we've seen the choice line

        def read_output():
            nonlocal approval_id, pending_description, pending_command_lines, waiting_for_choice

            for line in process.stdout:
                stripped = line.rstrip('\n')
                output_lines.append(stripped)
                print(f"[hermes] {stripped}")

                # Emit activity lines for real-time UI feedback (before approval detection)
                if not waiting_for_choice:
                    activity = self._activity_from_output(stripped)
                    if activity and not (pending_description and not self._APPROVAL_CHOICES_RE.search(stripped) and not self._APPROVAL_PROMPT_RE.search(stripped)):
                        # Don't emit activity lines that are part of approval command text collection
                        # (between DANGEROUS COMMAND header and choices menu)
                        if not pending_description or self._APPROVAL_START_RE.search(stripped):
                            self._emit_activity(activity)

                # Detect the start of a dangerous-command approval block
                m = self._APPROVAL_START_RE.search(stripped)
                if m and not waiting_for_choice:
                    pending_description = m.group(1).strip()
                    pending_command_lines = []
                    approval_id = uuid.uuid4().hex
                    print(f"🔐 Approval needed ({approval_id[:8]}): {pending_description}")
                    continue

                # Collect command lines between the description and the choices menu
                if pending_description and not waiting_for_choice:
                    if self._APPROVAL_CHOICES_RE.search(stripped) or self._APPROVAL_PROMPT_RE.search(stripped):
                        # Choices menu OR prompt line reached.
                        # Hermes prints "Choice [o/s/a/D]:" with newline, then blocks on readline().
                        # We can trigger on either line for robustness.
                        command_text = "\n".join(
                            l.strip() for l in pending_command_lines if l.strip()
                        ) or "(unknown command)"

                        # Register event before emitting so race-free
                        event = threading.Event()
                        self._approval_events[approval_id] = event

                        self._emit_approval_request(approval_id, command_text, pending_description)
                        waiting_for_choice = True

                        # Block this reader thread until user responds
                        resolved = event.wait(timeout=APPROVAL_TIMEOUT_SEC)
                        choice = self._approval_choices.pop(approval_id, None) if resolved else None
                        self._approval_events.pop(approval_id, None)

                        if not resolved or choice is None or choice not in self._CHOICE_TO_KEY:
                            key = "d"  # default deny on timeout/invalid
                            reason = "timed out" if not resolved else f"invalid choice: {choice}"
                            print(f"⏱️ Approval {approval_id[:8]} {reason} — denying")
                            self._emit_error(f"[危险命令授权{reason}，已自动拒绝]")
                        else:
                            key = self._CHOICE_TO_KEY[choice]
                            print(f"🔐 Writing choice '{key}' to Hermes stdin")

                        # Clear the approval card from the UI
                        self._emit_approval_cleared()

                        # Write the choice to the process stdin
                        try:
                            process.stdin.write(key + "\n")
                            process.stdin.flush()
                        except (BrokenPipeError, OSError) as e:
                            print(f"⚠️ Could not write to Hermes stdin: {e}")

                        # Reset state for potential next approval in same run
                        pending_description = None
                        pending_command_lines = []
                        waiting_for_choice = False
                        approval_id = None
                    elif stripped.strip():
                        # Collect command text (any non-empty line between
                        # the DANGEROUS COMMAND header and the choices menu)
                        pending_command_lines.append(stripped)

        reader_thread = threading.Thread(target=read_output, daemon=True)
        reader_thread.start()

        try:
            process.wait(timeout=CLI_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            # Kill any pending approval event so the reader thread unblocks
            for evt in list(self._approval_events.values()):
                evt.set()
            process.kill()
            process.wait()
            return -1, "\n".join(output_lines)
        finally:
            # Clear process reference
            with self._process_lock:
                self._current_process = None
            try:
                process.stdin.close()
            except Exception:
                pass

        reader_thread.join(timeout=5)
        return process.returncode, "\n".join(output_lines)

    def interrupt_current(self) -> bool:
        """Interrupt the currently running Hermes CLI process.

        Sends SIGINT (Ctrl+C) to the process. Returns True if interrupted,
        False if no process was running.
        """
        with self._process_lock:
            process = self._current_process
            if process is None:
                print("⚠️ No Hermes CLI process running to interrupt")
                return False

            print(f"🔴 Interrupting Hermes CLI (PID: {process.pid})")
            try:
                # Send SIGINT (Ctrl+C equivalent)
                process.send_signal(subprocess.signal.SIGINT)
                # Also clear any pending approval events
                for evt in list(self._approval_events.values()):
                    evt.set()
                self._approval_events.clear()
                self._approval_choices.clear()
                return True
            except Exception as e:
                print(f"⚠️ Failed to interrupt: {e}")
                # Fallback: terminate
                process.terminate()
                return True

    def _cli_worker(self):
        """Worker thread that processes messages serially."""
        while self._running:
            try:
                # Block until message available
                content, client_msg_id = self._msg_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # Mark busy FIRST so finally always runs
            self._busy = True
            self._emit_status("busy")

            try:
                # Decide whether to create a new session or resume existing
                create_new = self._new_session_pending or self._active_session is None

                if not create_new and not self._active_session:
                    self._emit_error("[错误: 没有活跃的 Hermes 会话]")
                    continue  # Will go to finally

                # Record the max message ID before CLI call
                before_max_id = self._last_id  # noqa: F841 (reserved for future use)

                if create_new:
                    print(f"📤 Creating NEW Hermes session with: {content[:50]}...")
                    cmd = [HERMES_CLI, 'chat', '-Q', '-q', content]
                else:
                    print(f"📤 Resuming Hermes session {self._active_session}: {content[:50]}...")
                    cmd = [HERMES_CLI, 'chat', '-Q', '--resume', self._active_session, '-q', content]

                # Clear proxy vars since OpenAI client doesn't support socks://
                env = {**os.environ, 'TERM': 'dumb'}
                for proxy_var in ['http_proxy', 'https_proxy', 'HTTP_PROXY',
                                  'HTTPS_PROXY', 'all_proxy', 'ALL_PROXY']:
                    env.pop(proxy_var, None)

                returncode, output = self._run_cli_with_approval(cmd, env)

                if returncode == 0:
                    print(f"✅ Hermes CLI accepted message")

                    # If we created a new session, discover its session_id from DB
                    if create_new:
                        try:
                            conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
                            cursor = conn.cursor()
                            cursor.execute("""
                                SELECT session_id FROM messages
                                WHERE role IN ('user', 'assistant')
                                ORDER BY id DESC LIMIT 1
                            """)
                            row = cursor.fetchone()
                            if row:
                                self._active_session = row[0]
                                self._new_session_pending = False
                                print(f"🆕 New active session: {self._active_session}")
                            conn.close()
                        except Exception as e:
                            print(f"⚠️ Failed to discover new session: {e}")
                    # DB poll will pick up the response using _last_id

                elif returncode == -1:
                    print(f"⏱️ Hermes CLI timeout after {CLI_TIMEOUT_SEC}s")
                    self._emit_error(f"[超时: Hermes 响应时间过长 ({CLI_TIMEOUT_SEC}s)]")
                else:
                    error_msg = output.strip() or f"exit code {returncode}"
                    # Truncate very long output
                    if len(error_msg) > 300:
                        error_msg = error_msg[-300:]
                    print(f"❌ Hermes CLI error (code {returncode}): {error_msg[:100]}")
                    self._emit_error(f"[发送失败: {error_msg[:200]}]")

            except FileNotFoundError:
                print(f"❌ Hermes CLI not found: {HERMES_CLI}")
                self._emit_error(f"[错误: 找不到 Hermes CLI]")

            except Exception as e:
                print(f"❌ Unexpected error: {e}")
                self._emit_error(f"[错误: {str(e)[:100]}]")

            finally:
                # Always mark idle and task_done, regardless of error path
                # Also clear any leftover approval state
                for evt in list(self._approval_events.values()):
                    evt.set()
                self._approval_events.clear()
                self._approval_choices.clear()
                self._emit_approval_cleared()
                self._busy = False
                self._emit_status("idle")
                self._msg_queue.task_done()


# Create Socket.IO server
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")
bridge = HermesBridge(sio)


@sio.event
async def connect(sid, environ):
    print(f"🔗 UI Connected to Hermes Bridge: {sid}")
    await bridge.send_history(sid)
    # Send current status
    await sio.emit("hermes_status", {"status": "busy" if bridge._busy else "idle"}, room=sid)


@sio.event
async def new_session(sid, data=None):
    """Handle new session request from UI."""
    if bridge._busy:
        print(f"⚠️ New session rejected: Hermes is busy")
        await sio.emit("new_session_ack", {"status": "rejected", "reason": "busy"}, room=sid)
        return

    # Drain any queued messages
    while not bridge._msg_queue.empty():
        try:
            bridge._msg_queue.get_nowait()
            bridge._msg_queue.task_done()
        except queue.Empty:
            break

    # Set _last_id to current max to ignore all old messages
    try:
        conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
        cursor = conn.cursor()
        cursor.execute("SELECT MAX(id) FROM messages")
        row = cursor.fetchone()
        if row and row[0]:
            bridge._last_id = row[0]
        conn.close()
    except Exception as e:
        print(f"⚠️ Failed to update _last_id: {e}")

    bridge._active_session = None
    bridge._new_session_pending = True
    print(f"🆕 New session requested by {sid}, pending first message")
    await sio.emit("new_session_ack", {"status": "ok"}, room=sid)


@sio.event
async def send_message(sid, data):
    """Handle incoming message from UI."""
    content = data.get("content", "").strip()
    client_msg_id = data.get("clientMessageId")
    if content:
        print(f"📥 UI -> Hermes: {content[:50]}...")
        bridge.queue_message(content, client_msg_id)


@sio.event
async def approval_response(sid, data):
    """Handle user's approval decision for a dangerous command."""
    approval_id = data.get("id", "")
    choice = data.get("choice", "deny")
    valid_choices = {"once", "session", "always", "deny"}
    if choice not in valid_choices:
        choice = "deny"
    print(f"🔐 Approval response from {sid}: {approval_id[:8]} → {choice}")
    bridge.resolve_approval(approval_id, choice)


@sio.event
async def interrupt_hermes(sid, data=None):
    """Handle interrupt request from UI (pause task button)."""
    print(f"🔴 Interrupt request from {sid}")
    if bridge.interrupt_current():
        # Emit confirmation to UI
        await sio.emit("hermes_interrupted", {"status": "ok"}, room=sid)
    else:
        await sio.emit("hermes_interrupted", {"status": "no_process"}, room=sid)


def create_app():
    """Create the Starlette ASGI application."""
    async def health(request):
        return Response(content="OK", media_type="text/plain")

    routes = [Route("/health", health)]
    starlette_app = Starlette(routes=routes)
    return socketio.ASGIApp(sio, starlette_app)


def start_broadcast_loop():
    """Start the asyncio event loop for broadcasting."""
    bridge._loop = asyncio.new_event_loop()
    asyncio.set_event_loop(bridge._loop)
    try:
        bridge._loop.run_forever()
    except Exception as e:
        print(f"Broadcast loop error: {e}")
    finally:
        bridge._loop.close()


if __name__ == "__main__":
    import signal

    # Start broadcast loop in a thread
    broadcast_thread = threading.Thread(target=start_broadcast_loop, daemon=True)
    broadcast_thread.start()

    # Start DB polling thread
    threading.Thread(target=bridge.poll_db, daemon=True).start()

    print("🚀 Hermes Bridge running on port 7780")
    print(f"   DB: {DB_PATH}")
    print(f"   CLI: {HERMES_CLI}")
    print(f"   Timeout: {CLI_TIMEOUT_SEC}s")

    # Run the ASGI app with uvicorn
    app = create_app()
    config = uvicorn.Config(app, host="0.0.0.0", port=7780, log_level="error")
    server = uvicorn.Server(config)

    def handle_shutdown(signum=None, frame=None):
        print("\nShutting down Hermes Bridge...")
        server.should_exit = True

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    server.run()
