import asyncio
from bleak import BleakScanner, BleakClient

TARGET_ADDRESS = "98:A3:16:B1:D0:CE"
CHARACTERISTIC_UUID = "abcdefab-1234-1234-1234-abcdefabcdef"


def notification_handler(sender, data):
    try:
        print("[Notification]", data.decode("utf-8"))
    except UnicodeDecodeError:
        print("[Notification raw]", data)


async def main():
    print(f"Connecting to {TARGET_ADDRESS}...")

    async with BleakClient(TARGET_ADDRESS) as client:
        print("Connected:", client.is_connected)

        print("Starting notifications...")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        print("Reading initial value...")
        value = await client.read_gatt_char(CHARACTERISTIC_UUID)
        try:
            print("Initial read:", value.decode("utf-8"))
        except UnicodeDecodeError:
            print("Initial read raw:", value)

        print("Type commands to send to the ESP32.")
        print("Examples: forward, left, right, stop")
        print("Type 'quit' to exit.")

        while True:
            msg = input("Send> ").strip()
            if msg.lower() == "quit":
                break
            if not msg:
                continue

            await client.write_gatt_char(
                CHARACTERISTIC_UUID,
                msg.encode("utf-8"),
                response=True
            )

        print("Stopping notifications...")
        await client.stop_notify(CHARACTERISTIC_UUID)

    print("Disconnected.")


if __name__ == "__main__":
    asyncio.run(main())