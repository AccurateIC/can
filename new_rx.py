#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial

class SBUSReceiver:
    START_BYTE = 0x0F
    SBUS_FRAME_LEN = 25

    def __init__(self, port='/dev/ttyUSB0', baudrate=100000):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self._in_frame = False
        self._frame = bytearray()

    def read_sbus_frame(self):
        while True:
            byte = self.serial_port.read()  # Read a single byte
            
            if not byte:  # If no byte is received, continue the loop
                continue
            
            inverted_byte = ~byte[0] & 0xFF  # Invert the byte received
            
            if self._in_frame:
                print(byte)
                self._frame.append(inverted_byte)
                if len(self._frame) == SBUSReceiver.SBUS_FRAME_LEN:
                    return self.process_frame(self._frame)
            else:
                print(byte," ", SBUSReceiver.START_BYTE)
                if inverted_byte == SBUSReceiver.START_BYTE:
                    print(byte)
                    self._in_frame = True
                    self._frame.clear()
                    self._frame.append(inverted_byte)
        
    def process_frame(self, frame):
        # Assuming that the frame is valid, parse it here
        sbus_channels = [None] * 18
        channel_sum = int.from_bytes(frame[1:23], byteorder="little")

        for ch in range(0, 16):
            sbus_channels[ch] = channel_sum & 0x7FF
            channel_sum >>= 11

        # Handle channels 17 and 18
        sbus_channels[16] = 2047 if (frame[23] & 0x01) else 0
        sbus_channels[17] = 2047 if ((frame[23] >> 1) & 0x01) else 0

        # Failsafe status
        failsafe_status = "OK"
        if (frame[SBUSReceiver.SBUS_FRAME_LEN - 2]) & (1 << 2):
            failsafe_status = "LOST"
        if (frame[SBUSReceiver.SBUS_FRAME_LEN - 2]) & (1 << 3):
            failsafe_status = "FAILSAFE"

        return sbus_channels, failsafe_status

def main():
    receiver = SBUSReceiver('/dev/ttyTHS0')
    print("Yo")
    while True:
        frame_data, status = receiver.read_sbus_frame()
        print(f"Channels: {frame_data}, Failsafe Status: {status}")

if __name__ == '__main__':
    main()
