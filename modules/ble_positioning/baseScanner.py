import asyncio
from bleak import BleakScanner
from bleak.backends.scanner import AdvertisementData

flag=False

def detection_callback(device, advertisement_data: AdvertisementData):
    if advertisement_data.service_uuids:
        print(f"\n📡 Detected device: {device.address}")
        print(f"🔹 Name             : {device.name or advertisement_data.local_name or 'Unknown'}")
        print(f"🔹 RSSI             : {device.rssi} dBm")
        print(f"🔹 TX Power         : {advertisement_data.tx_power or 'N/A'} dBm")
        print(f"🔹 Local Name       : {advertisement_data.local_name}")
        print(f"🔹 Manufacturer Data: {advertisement_data.manufacturer_data}")
        print(f"🔹 Service UUIDs    : {advertisement_data.service_uuids}")
        print(f"🔹 Service Data     : {advertisement_data.service_data}")
        print(f"🔹 Platform Metadata: {advertisement_data.platform_data}")
        print("-" * 50)
    

async def main(scan_time=10):
    scanner = BleakScanner()
    scanner.register_detection_callback(detection_callback)

    print(f"🔍 Scanning for {scan_time} seconds...")
    await scanner.start()
    await asyncio.sleep(scan_time)
    await scanner.stop()
    print("✅ Scan complete.")

    print(flag)

if __name__ == "__main__":
    asyncio.run(main())
