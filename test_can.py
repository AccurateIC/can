import os
import time

def send_alternating_bits():
    frames = [
        "sudo cansend can0 123##155555555",  # 0x55 pattern: 01010101
        "sudo cansend can0 123##1AAAAAAAA"   # 0xAA pattern: 10101010
    ]
    
    while True:
        for frame in frames:
            os.system(frame)
            time.sleep(1)

if __name__ == "__main__":
    send_alternating_bits()

