import serial
import logging
import time

# SBUS constants
SBUS_FRAME_SIZE = 25
SBUS_BAUD_RATE = 100000
SBUS_HEADER = 0x0F
SBUS_INVERTED_HEADER = 0xF8
SBUS_FOOTER = 0x00
SBUS_FLAGS_BYTE = 23
SBUS_NUM_CHANNELS = 16

# SBUS frame masks for channels
SBUS_CHANNEL_MASKS = [
    (0x07FF, 0), (0x07FF, 11), (0x07FF, 22), (0x07FF, 33),
    (0x07FF, 44), (0x07FF, 55), (0x07FF, 66), (0x07FF, 77),
    (0x07FF, 88), (0x07FF, 99), (0x07FF, 110), (0x07FF, 121),
    (0x07FF, 132), (0x07FF, 143), (0x07FF, 154), (0x07FF, 165),
]

# Function to map SBUS channel values to a calibrated range
def map_sbus_channel_value(raw_value, in_min=133, in_max=1811, out_min=0, out_max=1000):
    """
    Maps the SBUS raw values (usually between 172 and 1811) to a new range.
    :param raw_value: The raw SBUS value from the channel
    :param in_min: The minimum value of the SBUS input range (default 172)
    :param in_max: The maximum value of the SBUS input range (default 1811)
    :param out_min: The minimum value of the output range (default 0)
    :param out_max: The maximum value of the output range (default 1000)
    :return: Calibrated value mapped to the new range
    """
    return (raw_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Function to invert signal (bit inversion)
def invert_signal(byte):
    return byte ^ 0xFF  # XOR with 0xFF to flip all bits

def decode_sbus_frame_bitwise(frame):
    channels = [0] * SBUS_NUM_CHANNELS

    # Initialize bit counters
    byte_in_sbus = 1  # Start after the header byte
    bit_in_sbus = 0
    ch = 0
    bit_in_channel = 0

    # Iterate over 176 bits (16 channels x 11 bits each)
    for i in range(0, 176):
        # Extract bit from SBUS frame
        if frame[byte_in_sbus] & (1 << bit_in_sbus):
            # Set corresponding bit in the channel value
            channels[ch] |= (1 << bit_in_channel)

        # Move to the next bit in the SBUS frame
        bit_in_sbus += 1
        bit_in_channel += 1

        # If we've processed 8 bits, move to the next byte
        if bit_in_sbus == 8:
            bit_in_sbus = 0
            byte_in_sbus += 1

        # Each channel is 11 bits, so reset after 11 bits and move to the next channel
        if bit_in_channel == 11:
            bit_in_channel = 0
            ch += 1

    # Get additional data from the flags byte (Digital Channels and Failsafe)
    digital_channel_17 = (frame[SBUS_FLAGS_BYTE] & 0x80) >> 7
    digital_channel_18 = (frame[SBUS_FLAGS_BYTE] & 0x40) >> 6
    frame_lost = (frame[SBUS_FLAGS_BYTE] & 0x20) >> 5
    failsafe_active = (frame[SBUS_FLAGS_BYTE] & 0x10) >> 4

    return channels, digital_channel_17, digital_channel_18, frame_lost, failsafe_active

# Decode SBUS frame into channel values
def decode_sbus_frame(frame):
    channels = [0] * SBUS_NUM_CHANNELS
    for i in range(SBUS_NUM_CHANNELS):
        bitmask, shift = SBUS_CHANNEL_MASKS[i]
        channels[i] = (
            (frame[1 + (shift // 8)] | (frame[2 + (shift // 8)] << 8)) >> (shift % 8)
        ) & bitmask

    # Get additional data from the flags byte
    digital_channel_17 = (frame[SBUS_FLAGS_BYTE] & 0x80) >> 7
    digital_channel_18 = (frame[SBUS_FLAGS_BYTE] & 0x40) >> 6
    frame_lost = (frame[SBUS_FLAGS_BYTE] & 0x20) >> 5
    failsafe_active = (frame[SBUS_FLAGS_BYTE] & 0x10) >> 4

    return channels, digital_channel_17, digital_channel_18, frame_lost, failsafe_active

# Function to find a valid SBUS frame by syncing on header and footer
def find_valid_sbus_frame(serial_port):
    buffer = bytearray()

    while True:
        byte = serial_port.read()  # Read one byte at a time
        #print(byte)
        buffer.append(ord(byte))  # Add the byte to the buffer

        # Check if we have enough bytes for a full frame
        if len(buffer) >= SBUS_FRAME_SIZE:
            # Check for inverted SBUS header (0xF8) or normal header (0x0F)
            if buffer[0] == SBUS_INVERTED_HEADER or buffer[0] == SBUS_HEADER:
                # Invert the frame if the header is 0xF8
                if buffer[0] == SBUS_INVERTED_HEADER:
                    # frame = bytearray([invert_signal(b) for b in buffer])
                    frame = buffer
                else:
                    frame = buffer

                # Validate the footer (last byte should be 0x00)
                if frame[-1] == SBUS_FOOTER:
                    channels, digital_17, digital_18, frame_lost, failsafe_active = decode_sbus_frame_bitwise(frame)
                    calibrated_channels = [map_sbus_channel_value(ch) for ch in channels]

                    # Print channel values and flags
                    logging.info("Channels: %s", calibrated_channels[2])
                    logging.info("Digital Channel 17: %d", digital_17)
                    logging.info("Digital Channel 18: %d", digital_18)
                    logging.info("Frame Lost: %d", frame_lost)
                    logging.info("Failsafe Active: %d", failsafe_active)
                    logging.info("------------------------")

                    return frame  # Valid frame found

            # If no valid frame, shift buffer to check the next sequence
            buffer.pop(0)

# Main function to read SBUS frames
def read_sbus():
    try:
        # Open serial port for FT232RL
        ser = serial.Serial(
            port='/dev/ttyTHS0',  # Change this to your FT232RL's port
            baudrate=SBUS_BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,  # SBUS uses even parity
            stopbits=serial.STOPBITS_TWO,  # SBUS uses 2 stop bits
            timeout=1  # Add a timeout for read operations
        )
    except serial.SerialException as e:
        logging.error(f"Failed to open serial port: {e}")
        return

    logging.info("Waiting for SBUS data...")

    # Continuously read and decode SBUS frames
    while True:
        frame = find_valid_sbus_frame(ser)  # Search for a valid SBUS frame

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    read_sbus()
