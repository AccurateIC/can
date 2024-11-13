import serial
import logging

# SBUS constants
SBUS_FRAME_SIZE = 25
SBUS_BAUD_RATE = 100000
SBUS_HEADER = 0x0F
SBUS_INVERTED_HEADER = 0xF8
SBUS_FOOTER = 0x00
SBUS_FLAGS_BYTE = 23
SBUS_NUM_CHANNELS = 16

# Function to map SBUS channel values to a calibrated range
def map_sbus_channel_value(raw_value, in_min=172, in_max=1811, out_min=0, out_max=1000):
    """Maps the SBUS raw values to a new range."""
    return (raw_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Function to invert signal (bit inversion)
def invert_signal(byte):
    return byte ^ 0xFF  # XOR with 0xFF to flip all bits

# Function to decode SBUS frame into channel values
def decode_sbus_frame(frame):
    channels = [0] * SBUS_NUM_CHANNELS

    # Extract channels from the frame
    channels[0] = (frame[1] | (frame[2] << 8)) & 0x07FF  # Channel 1
    channels[1] = ((frame[2] >> 3) | (frame[3] << 5)) & 0x07FF  # Channel 2
    channels[2] = ((frame[3] >> 6) | (frame[4] << 2)) & 0x07FF  # Channel 3
    channels[2] |= (frame[5] << 10) & 0x07FF  # Channel 3 continued
    channels[3] = (frame[5] >> 1) & 0x07FF  # Channel 4
    channels[4] = ((frame[5] >> 4) | (frame[6] << 4)) & 0x07FF  # Channel 5
    channels[5] = ((frame[6] >> 4) | (frame[7] << 6)) & 0x07FF  # Channel 6
    channels[6] = ((frame[7] >> 2) | (frame[8] << 3)) & 0x07FF  # Channel 7
    channels[6] |= (frame[9] << 11) & 0x07FF  # Channel 7 continued
    channels[7] = (frame[9] >> 1) & 0x07FF  # Channel 8
    channels[8] = ((frame[9] >> 4) | (frame[10] << 4)) & 0x07FF  # Channel 9
    channels[9] = ((frame[10] >> 4) | (frame[11] << 6)) & 0x07FF  # Channel 10
    channels[10] = ((frame[11] >> 2) | (frame[12] << 3)) & 0x07FF  # Channel 11
    channels[10] |= (frame[13] << 11) & 0x07FF  # Channel 11 continued
    channels[11] = (frame[13] >> 1) & 0x07FF  # Channel 12
    channels[12] = ((frame[13] >> 4) | (frame[14] << 4)) & 0x07FF  # Channel 13
    channels[13] = ((frame[14] >> 4) | (frame[15] << 6)) & 0x07FF  # Channel 14
    channels[14] = ((frame[15] >> 2) | (frame[16] << 3)) & 0x07FF  # Channel 15
    channels[14] |= (frame[17] << 11) & 0x07FF  # Channel 15 continued
    channels[15] = (frame[17] >> 1) & 0x07FF  # Channel 16

    # Log raw channels before calibration
    logging.info("Raw Channels (before calibration): %s", channels)

    # Get additional data from the flags byte
    digital_channel_17 = (frame[SBUS_FLAGS_BYTE] & 0x80) >> 7
    digital_channel_18 = (frame[SBUS_FLAGS_BYTE] & 0x40) >> 6
    frame_lost = (frame[SBUS_FLAGS_BYTE] & 0x20) >> 5
    failsafe_active = (frame[SBUS_FLAGS_BYTE] & 0x10) >> 4

    return channels, digital_channel_17, digital_channel_18, frame_lost, failsafe_active

# Function to find a valid SBUS frame by syncing on header and footer
def find_valid_sbus_frame(serial_port):
    buffer = bytearray()
    print(buffer)
    while True:
        chunk = serial_port.read(1)  # Read 32 bytes at a time
        if not chunk:  # If no bytes read, continue
            continue
        
        buffer.extend(chunk)

        while len(buffer) >= SBUS_FRAME_SIZE:
            if buffer[0] == SBUS_INVERTED_HEADER:
                frame = bytearray(invert_signal(b) for b in buffer[:SBUS_FRAME_SIZE])
            elif buffer[0] == SBUS_HEADER:
                frame = buffer[:SBUS_FRAME_SIZE]
            else:
                buffer.pop(0)
                continue

            if frame[-1] == SBUS_FOOTER:
                channels, digital_17, digital_18, frame_lost, failsafe_active = decode_sbus_frame(frame)
                
                # Log calibrated channels
                calibrated_channels = [map_sbus_channel_value(ch) for ch in channels]
                logging.info("Calibrated Channels: %s", calibrated_channels)
                
                # Additional logging
                logging.info("Digital Channel 17: %d", digital_17)
                logging.info("Digital Channel 18: %d", digital_18)
                logging.info("Frame Lost: %d", frame_lost)
                logging.info("Failsafe Active: %d", failsafe_active)
                logging.info("------------------------")

                # Remove processed bytes from the buffer
                buffer = buffer[SBUS_FRAME_SIZE:]
                return frame
            
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
        find_valid_sbus_frame(ser)  # Search for a valid SBUS frame

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    read_sbus()
