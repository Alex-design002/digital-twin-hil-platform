#!/usr/bin/env python3
"""Wi-Fi Web Dashboard Server for Traxxas TRX-4 HIL Controller.

Serves ``trx4_web_ui.html`` and JSON/SSE control endpoints using Python's
standard library (zero external web dependencies) so a computer on the same
Wi-Fi network or hotspot can select scenarios, adjust terrain/weather
conditions, override governor modes, and monitor live telemetry.
"""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import platform
import socket
import subprocess
import sys
import threading
from typing import Any, Optional, Sequence
from urllib.parse import urlparse


UI_HTML_PATH = Path(__file__).resolve().parent / "trx4_web_ui.html"
DEFAULT_WEB_HOST = "0.0.0.0"
DEFAULT_WEB_PORT = 5000


def discover_wifi_urls(bind_host: str, port: int) -> list[str]:
    """Return likely browser URLs reachable from another computer over Wi-Fi."""
    urls: list[str] = []
    seen: set[str] = set()

    def _add(host: str) -> None:
        h = host.strip()
        if not h or h in ("0.0.0.0", "::"):
            return
        url = f"http://{h}:{port}"
        if url not in seen:
            seen.add(url)
            urls.append(url)

    if bind_host not in ("0.0.0.0", "::", ""):
        _add(bind_host)

    hostname = socket.gethostname()
    if hostname:
        _add(f"{hostname}.local")

    # Primary outbound interface IP probe (does not transmit any packet).
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        _add(sock.getsockname()[0])
    except OSError:
        pass
    finally:
        sock.close()

    # Enumerate host IPv4 addresses across interfaces (useful on Raspberry Pi Wi-Fi).
    if platform.system() == "Linux":
        try:
            out = subprocess.check_output(["hostname", "-I"], text=True, timeout=1.0)
            for token in out.split():
                if "." in token and not token.startswith("127."):
                    _add(token)
        except Exception:
            pass

    try:
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                _add(ip)
    except OSError:
        pass

    _add("127.0.0.1")
    return urls


class _TRX4ThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        handler_cls: type[BaseHTTPRequestHandler],
        controller: Any,
        ui_path: Path,
        stop_event: threading.Event,
    ) -> None:
        self.controller = controller
        self.ui_path = ui_path
        self.stop_event = stop_event
        super().__init__(server_address, handler_cls)

    def handle_error(self, request: Any, client_address: Any) -> None:
        _exc_type, exc, _tb = sys.exc_info()
        if isinstance(exc, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError)):
            return
        super().handle_error(request, client_address)


class _TRX4RequestHandler(BaseHTTPRequestHandler):
    """REST + Server-Sent Events handler for TRX-4 Wi-Fi dashboard."""

    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        # Keep console output clean; controller logger handles structured logs.
        logging.getLogger("TRX4WebServer").debug(fmt, *args)

    def _send_bytes(
        self, status: int, content_type: str, body: bytes
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: int, payload: Any) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", raw)

    def _read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(min(length, 1024 * 64))
        if not raw.strip():
            return {}
        data = json.loads(raw.decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        return data

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send_bytes(HTTPStatus.NO_CONTENT, "text/plain; charset=utf-8", b"")

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        ctrl = self.server.controller  # type: ignore[attr-defined]

        if path in ("/", "/index.html"):
            ui_path: Path = self.server.ui_path  # type: ignore[attr-defined]
            if not ui_path.exists():
                self._send_json(
                    HTTPStatus.NOT_FOUND,
                    {"error": f"UI template not found: {ui_path}"},
                )
                return
            html_bytes = ui_path.read_bytes()
            self._send_bytes(
                HTTPStatus.OK, "text/html; charset=utf-8", html_bytes
            )
            return

        if path == "/api/state":
            self._send_json(HTTPStatus.OK, ctrl.get_state_snapshot())
            return

        if path == "/api/stream":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Private-Network", "true")
            self.end_headers()

            stop_evt: threading.Event = self.server.stop_event  # type: ignore[attr-defined]
            try:
                while not stop_evt.is_set():
                    snap = ctrl.get_state_snapshot()
                    chunk = (
                        "data: "
                        + json.dumps(snap, ensure_ascii=False)
                        + "\n\n"
                    ).encode("utf-8")
                    self.wfile.write(chunk)
                    self.wfile.flush()
                    if stop_evt.wait(0.15):
                        break
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass
            return

        self._send_json(HTTPStatus.NOT_FOUND, {"error": f"Unknown route: {path}"})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        ctrl = self.server.controller  # type: ignore[attr-defined]

        try:
            body = self._read_json_body()
        except Exception as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": f"Invalid JSON: {exc}"})
            return

        try:
            if path == "/api/scenario":
                action = body.get("action")
                if action == "next":
                    sc = ctrl.next_scenario()
                elif action == "prev":
                    sc = ctrl.prev_scenario()
                elif "id" in body:
                    sc = ctrl.select_scenario(int(body["id"]), by_id=True)
                else:
                    raise ValueError("Provide 'id' or 'action' ('next'/'prev')")
                ctrl.refresh_telemetry_preview()
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "selected_id": sc.scenario_id,
                        "state": ctrl.get_state_snapshot(),
                    },
                )
                return

            if path == "/api/conditions":
                sc = ctrl.set_custom_conditions(body)
                ctrl.refresh_telemetry_preview()
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "track_name": sc.track_name,
                        "state": ctrl.get_state_snapshot(),
                    },
                )
                return

            if path == "/api/control":
                ctrl.apply_remote_control(
                    control_mode=body.get("control_mode"),
                    use_live_imu=body.get("use_live_imu"),
                    buzzer_mode=body.get("buzzer_mode"),
                )
                ctrl.refresh_telemetry_preview()
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "state": ctrl.get_state_snapshot()},
                )
                return

            if path == "/api/reload":
                ctrl.load_physics(ctrl.config.excel_path)
                ctrl.refresh_telemetry_preview()
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "state": ctrl.get_state_snapshot()},
                )
                return

            if path == "/api/model":
                if "custom_specs" in body and isinstance(body["custom_specs"], dict):
                    ctrl.set_custom_specs(body["custom_specs"])
                elif "model_id" in body:
                    ctrl.select_model(str(body["model_id"]))
                else:
                    raise ValueError("Provide 'model_id' or 'custom_specs'")
                ctrl.refresh_telemetry_preview()
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "state": ctrl.get_state_snapshot()},
                )
                return

            self._send_json(HTTPStatus.NOT_FOUND, {"error": f"Unknown route: {path}"})
        except (ValueError, KeyError, TypeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
        except Exception as exc:
            self._send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR, {"error": f"Server error: {exc}"}
            )


class TRX4WebDashboardServer:
    """Lifecycle manager for the background Wi-Fi HTTP/SSE dashboard server."""

    def __init__(
        self,
        controller: Any,
        host: str = DEFAULT_WEB_HOST,
        port: int = DEFAULT_WEB_PORT,
        ui_path: Path = UI_HTML_PATH,
    ) -> None:
        self.controller = controller
        self.host = host
        self.port = int(port)
        self.ui_path = ui_path
        self._stop_event = threading.Event()
        self._httpd: Optional[_TRX4ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> list[str]:
        """Start the HTTP server in a background daemon thread and return URLs."""
        if self._httpd is not None:
            return self.urls
        self._stop_event.clear()
        self._httpd = _TRX4ThreadingHTTPServer(
            (self.host, self.port),
            _TRX4RequestHandler,
            controller=self.controller,
            ui_path=self.ui_path,
            stop_event=self._stop_event,
        )
        # Support ephemeral port=0 cleanly in tests.
        self.port = int(self._httpd.server_address[1])
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="TRX4WebDashboardHTTP",
            daemon=True,
        )
        self._thread.start()
        return self.urls

    @property
    def urls(self) -> list[str]:
        return discover_wifi_urls(self.host, self.port)

    def stop(self) -> None:
        """Stop SSE streams and shut down the HTTP server cleanly."""
        self._stop_event.set()
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass
            self._httpd = None
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Convenience entrypoint delegating to ``trx4_hil_controller`` with ``--web``."""
    from trx4_hil_controller import main as controller_main

    args = list(sys.argv[1:] if argv is None else argv)
    if "--web" not in args:
        args.insert(0, "--web")
    return controller_main(args)


if __name__ == "__main__":
    sys.exit(main())
