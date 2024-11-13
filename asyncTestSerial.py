import serial
import asyncio

async def receive_data():
    # Open the serial port (replace ttyTHS0 with your UART port)
    ser = serial.Serial('/dev/ttyTHS0', 115200, timeout=1)

    # Give some time for the connection to be established
    await asyncio.sleep(2)

    while True:
        if ser.in_waiting > 0:
            # Read incoming data from the serial port
            data = ser.readline().decode().strip()
            if data:
                print(f"Received: {data}")
        await asyncio.sleep(0.1)  # Sleep to prevent busy-waiting

if __name__ == "__main__":
    asyncio.run(receive_data())
