import asyncio
import struct
import json
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from bleak import BleakClient
import websockets

# Import your math and packet builder from your FBD file
from fbd_calculator import (
    SmoothedAcceleration,
    AccelerometerCalibration,
    raw_to_acceleration_g,
    build_fbd_packet,
    packet_to_json,
    MASS_KG,
    ADC_MIDPOINT,
)

# --- CONFIGURATION ---
DEVICE_ADDRESS = "E0:72:A1:AF:90:B2"
CHARACTERISTIC_UUID = "0000FF01-0000-1000-8000-00805F9B34FB"

WS_HOST = "0.0.0.0"
WS_PORT = 8765
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 3000

# --- GLOBAL STATES ---
connected_clients = set()
is_calibrating = True
calibration_samples = []
calibration_data = None
smoother = SmoothedAcceleration()
event_loop = None


# ============================================================
# WEBSOCKET BROADCASTER
# ============================================================
async def ws_handler(websocket, path="/"):
    """Handles new WebSocket connections from the web browser."""
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.remove(websocket)


async def broadcast_packet(json_str):
    """Sends the latest physics packet to all open web browsers."""
    if not connected_clients:
        return
    tasks = [asyncio.create_task(client.send(json_str)) for client in connected_clients]
    await asyncio.wait(tasks)


# ============================================================
# BLUETOOTH RECEIVER & PHYSICS ENGINE
# ============================================================
def notification_handler(sender, data):
    """Fired every time the ESP32 sends a new BLE packet."""
    global is_calibrating, calibration_samples, calibration_data, smoother, event_loop

    try:
        x_raw, y_raw, z_raw = struct.unpack("<fff", data)
    except struct.error:
        return

    # Phase 1: Calibration
    if is_calibrating:
        calibration_samples.append((x_raw, y_raw, z_raw))
        print(".", end="", flush=True)
        return

    # Phase 2: Live Physics Calculation
    if calibration_data:
        # 1. Convert to G-Force
        ax_g, ay_g, az_g = raw_to_acceleration_g(
            x_raw=x_raw, y_raw=y_raw, z_raw=z_raw, calibration=calibration_data
        )

        # 2. Smooth the data
        ax_g, ay_g, az_g = smoother.update(ax=ax_g, ay=ay_g, az=az_g, alpha=0.2)

        # 3. Build the packet
        packet = build_fbd_packet(
            ax_g=ax_g,
            ay_g=ay_g,
            az_g=az_g,
            calibration=calibration_data,
            mass_kg=MASS_KG,
        )

        # 4. Broadcast to Web Browser!
        json_packet = packet_to_json(packet)
        if event_loop and event_loop.is_running():
            asyncio.run_coroutine_threadsafe(broadcast_packet(json_packet), event_loop)


async def ble_task():
    global is_calibrating, calibration_samples, calibration_data

    print(f"\n[BLE] Connecting to {DEVICE_ADDRESS}...")
    try:
        async with BleakClient(DEVICE_ADDRESS) as client:
            print(
                "[BLE] Connected! Calibrating... Keep the sensor flat and perfectly still."
            )
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

            # Wait 2 seconds to collect calibration data
            await asyncio.sleep(2.0)

            is_calibrating = False
            if not calibration_samples:
                print("Error: No calibration samples received.")
                return

            # Perform the baseline math
            x0 = sum(s[0] for s in calibration_samples) / len(calibration_samples)
            y0 = sum(s[1] for s in calibration_samples) / len(calibration_samples)
            z0 = sum(s[2] for s in calibration_samples) / len(calibration_samples)

            deviations = {
                "x": x0 - ADC_MIDPOINT,
                "y": y0 - ADC_MIDPOINT,
                "z": z0 - ADC_MIDPOINT,
            }
            baseline_axis = max(deviations, key=lambda axis: abs(deviations[axis]))
            largest_deviation = deviations[baseline_axis]

            g_scale = abs(largest_deviation) if largest_deviation != 0 else 1
            baseline_sign = 1 if largest_deviation >= 0 else -1

            calibration_data = AccelerometerCalibration(
                x0=x0,
                y0=y0,
                z0=z0,
                g_scale=g_scale,
                midpoint=ADC_MIDPOINT,
                samples=len(calibration_samples),
                baseline_axis=baseline_axis,
                baseline_sign=baseline_sign,
            )

            print(
                f"\n[BLE] Calibration complete! Resting Avgs -> X: {x0:.1f}, Y: {y0:.1f}, Z: {z0:.1f}"
            )
            print("[BLE] Streaming Live Data to Website... Press Ctrl+C to stop.")

            while True:
                await asyncio.sleep(1)

    except Exception as e:
        print(f"\n[BLE] Error: {e}")


# ============================================================
# HTTP SERVER (HOSTS THE WEBSITE)
# ============================================================
class HttpHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            try:
                with open("index.html", "r") as f:
                    self.wfile.write(f.read().encode())
            except FileNotFoundError:
                self.wfile.write(
                    b"<h1>index.html not found!</h1><p>Please save your HTML file as 'index.html' in this folder.</p>"
                )
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress logging spam


def run_http_server():
    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), HttpHandler)
    server.serve_forever()


# ============================================================
# MAIN EVENT LOOP
# ============================================================
async def main():
    global event_loop
    event_loop = asyncio.get_running_loop()

    # 1. Start the HTTP server in a background thread
    threading.Thread(target=run_http_server, daemon=True).start()
    print(f"[HTTP] Web Dashboard running at: http://localhost:{HTTP_PORT}")

    # 2. Start the WebSocket server
    await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    print(f"[ WS ] WebSocket server running on port {WS_PORT}")

    # 3. Start connecting to Bluetooth
    await ble_task()


if __name__ == "__main__":
    asyncio.run(main())
