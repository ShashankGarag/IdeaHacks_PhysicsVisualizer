import asyncio
from bleak import BleakScanner


async def main():
    print("Scanning for Bluetooth devices...")
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == "Bluedroid_Conn":
            print(f"FOUND IT! Address: {d.address}")


asyncio.run(main())
