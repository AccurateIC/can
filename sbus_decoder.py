import serial
import struct

# SBUS Configuration
SERIAL_PORT = "/dev/ttyUSB0"  # Update this to your device's port (e.g., '/dev/ttyTHS1' for Jetson)
BAUD_RATE = 100000  # SBUS uses a baud rate of 100k
CHANNELS = 16

# SBUS Frame Information
SBUS_FRAME_LENGTH = 25  # SBUS frame length in bytes
START_BYTE = 0xF8
END_BYTE = 0x00

def decode_sbus_frame(data):
    """Decode SBUS frame data and return a list of 16 channel values."""
    channels = [0] * CHANNELS

    # Decode each 11-bit channel value
    channels[0] = ((data[1] | data[2] << 8) & 0x07FF)
    channels[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF)
    channels[2] = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF)
    channels[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF)
    channels[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF)
    channels[5] = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF)
    channels[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF)
    channels[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF)
    channels[8] = ((data[12] | data[13] << 8) & 0x07FF)
    channels[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF)
    channels[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF)
    channels[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF)
    channels[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF)
    channels[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF)
    channels[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF)
    channels[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF)

    # Return the decoded channels
    return channels

def main():
    # Open the UART port for reading
    with serial.Serial(SERIAL_PORT, BAUD_RATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO, timeout=0.1) as ser:
        print("Reading SBUS data...")

        while True:
            # Read 25 bytes (one SBUS frame)
            if ser.in_waiting >= SBUS_FRAME_LENGTH:
                frame = ser.read(SBUS_FRAME_LENGTH)
                print(frame)
                # Validate frame by checking start and end bytes
                if frame[0] == START_BYTE and frame[-1] == END_BYTE:
                    channels = decode_sbus_frame(frame)
                    print("Channel values:", channels)
                else:
                    print("Invalid SBUS frame received")
                    channels = decode_sbus_frame(frame)
                    print(frame)

if __name__ == "__main__":
    main()
