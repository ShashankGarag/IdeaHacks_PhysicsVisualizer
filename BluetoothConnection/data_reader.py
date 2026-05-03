import asyncio
import struct
from bleak import BleakClient

DEVICE_ADDRESS = "E0:72:A1:AF:90:B2"

CHARACTERISTIC_UUID = "0000FF01-0000-1000-8000-00805F9B34FB"


def notification_handler(sender, data):
    x, y, z = struct.unpack("<fff", data)
    print(f"X: {x:>5.2f} | Y: {y:>5.2f} | Z: {z:>5.2f}")


async def main():
    print(f"Connecting to {DEVICE_ADDRESS}...")
    try:
        async with BleakClient(DEVICE_ADDRESS) as client:
            print("Connected successfully!")

            # Turn on the notifications
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

            print("Listening for accelerometer data... Press Ctrl+C to stop.")
            # Keep the script alive forever to catch incoming data
            while True:
                await asyncio.sleep(1)

    except Exception as e:
        print(f"Connection failed: {e}")


asyncio.run(main())
