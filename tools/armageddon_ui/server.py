"""ARMageddon Control UI server.

Run:  python tools/armageddon_ui/server.py
      python tools/armageddon_ui/server.py --serial /dev/ttyACM0   (future)

Opens at http://localhost:8000
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from hardware import MODES, SENSITIVITY_DETENTS, ControlState, MockInterface, TeensySerialInterface

STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(title="ARMageddon Control UI")

# Global state
interface = MockInterface()
connected_clients: list[WebSocket] = []


async def broadcast(state: ControlState) -> None:
    """Send current state to all connected WebSocket clients."""
    payload = json.dumps({"type": "state", **state.to_dict()})
    stale = []
    for ws in connected_clients:
        try:
            await ws.send_text(payload)
        except Exception:
            stale.append(ws)
    for ws in stale:
        connected_clients.remove(ws)


def handle_event(event: dict, state: ControlState) -> ControlState:
    """Apply a UI event to the current state and return the updated state."""
    action = event.get("action")

    if action == "power_toggle":
        state.power = not state.power
        if state.power:
            state.mode = "joint"
            state.axis_index = 0
            state.jog_position = 0.0
            state.sensitivity_index = 5  # default 50%

    if not state.power:
        return state

    if action == "mode_cycle":
        idx = MODES.index(state.mode)
        state.mode = MODES[(idx + 1) % len(MODES)]
        state.axis_index = 0
        state.jog_position = 0.0

    elif action == "scroll_up":
        if state.mode != "leader":
            axes = state.axes
            if axes:
                state.axis_index = (state.axis_index + 1) % len(axes)

    elif action == "scroll_down":
        if state.mode != "leader":
            axes = state.axes
            if axes:
                state.axis_index = (state.axis_index - 1) % len(axes)

    elif action == "sensitivity_up":
        state.sensitivity_index = min(len(SENSITIVITY_DETENTS) - 1, state.sensitivity_index + 1)

    elif action == "sensitivity_down":
        state.sensitivity_index = max(0, state.sensitivity_index - 1)

    elif action == "jog_click":
        if state.mode != "leader":
            direction = event.get("direction", 1)  # +1 or -1
            state.jog_position += state.jog_step_size * direction

    return state


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    connected_clients.append(ws)
    # Send initial state
    state = await interface.read_state()
    await ws.send_text(json.dumps({"type": "state", **state.to_dict()}))
    try:
        while True:
            data = await ws.receive_text()
            event = json.loads(data)
            state = await interface.read_state()
            state = handle_event(event, state)
            await interface.on_state_change(state)
            await broadcast(state)
    except WebSocketDisconnect:
        connected_clients.remove(ws)


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


def main() -> None:
    parser = argparse.ArgumentParser(description="ARMageddon Control UI Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host")
    parser.add_argument("--port", type=int, default=8000, help="Bind port")
    parser.add_argument("--serial", default=None, help="Teensy serial port (e.g. /dev/ttyACM0)")
    args = parser.parse_args()

    global interface
    if args.serial:
        interface = TeensySerialInterface(args.serial)
    else:
        interface = MockInterface()

    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
