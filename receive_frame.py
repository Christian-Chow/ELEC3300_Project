import serial
import serial.tools.list_ports
import numpy as np
from PIL import Image
import time
import sys

# Configuration
SERIAL_PORT = 'COM3'  # Change to your port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 921600
FRAME_SIZE = 153600  # 320 * 240 * 2 bytes (RGB565)
WIDTH = 320
HEIGHT = 240

def main():
    print("=" * 50)
    print("STM32 Camera Frame Receiver")
    print("=" * 50)
    
    # Connect to serial port
    print(f"[INFO] Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=5)
        print(f"[SUCCESS] Connected to {SERIAL_PORT}")
        print(f"[INFO] Serial port settings: {ser}")
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open serial port: {e}")
        print(f"[INFO] Available ports:")
        for port in serial.tools.list_ports.comports():
            print(f"  - {port.device}: {port.description}")
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        sys.exit(1)
    
    frame_count = 0
    
    try:
        while True:
            print("\n" + "-" * 50)
            print(f"[INFO] Waiting for frame data... (Frame #{frame_count + 1})")
            
            # Wait for data
            start_time = time.time()
            print("[DEBUG] Checking for available data...")
            
            if ser.in_waiting == 0:
                print("[DEBUG] No data available, waiting...")
            
            # Read frame data
            print(f"[INFO] Reading {FRAME_SIZE} bytes from serial port...")
            data = ser.read(FRAME_SIZE)
            elapsed_time = time.time() - start_time
            
            if len(data) < FRAME_SIZE:
                print(f"[WARNING] Received only {len(data)} bytes, expected {FRAME_SIZE}")
                print(f"[INFO] This might be incomplete. Retrying...")
                continue
            
            print(f"[SUCCESS] Received {len(data)} bytes in {elapsed_time:.2f} seconds")
            print(f"[INFO] Transfer rate: {len(data)/elapsed_time/1024:.2f} KB/s")
            
            # Convert to numpy array
            print("[DEBUG] Converting bytes to numpy array...")
            # STM32 sends bytes as: High byte, Low byte (big-endian)
            # Convert to little-endian for proper interpretation
            pixels = np.frombuffer(data, dtype='>u2')  # Big-endian uint16
            print(f"[DEBUG] Array shape: {pixels.shape}, dtype: {pixels.dtype}")
            
            # Reshape to image dimensions
            print(f"[DEBUG] Reshaping array to ({HEIGHT}, {WIDTH})...")
            image_array = pixels.reshape(HEIGHT, WIDTH)
            print(f"[SUCCESS] Image array created: {image_array.shape}")
            
            # Convert RGB565 to RGB888 (vectorized for speed)
            print("[DEBUG] Converting RGB565 to RGB888...")
            # Extract color channels using bit operations
            r = ((image_array >> 11) & 0x1F) << 3
            g = ((image_array >> 5) & 0x3F) << 2
            b = (image_array & 0x1F) << 3
            # Stack into RGB image
            rgb_image = np.stack([r, g, b], axis=2).astype(np.uint8)
            print("[SUCCESS] Color conversion completed")
            
            # Create PIL Image
            print("[DEBUG] Creating PIL Image object...")
            img = Image.fromarray(rgb_image, 'RGB')
            print(f"[SUCCESS] Image created: {img.size}, mode: {img.mode}")
            
            # Save image
            filename = f"frame_{frame_count:04d}.jpg"
            print(f"[INFO] Saving image to '{filename}'...")
            img.save(filename, 'JPEG', quality=95)
            print(f"[SUCCESS] Image saved successfully!")
            
            # Optional: Display image info
            print(f"[INFO] Image statistics:")
            print(f"  - Size: {WIDTH}x{HEIGHT} pixels")
            print(f"  - File: {filename}")
            print(f"  - Format: JPEG")
            
            frame_count += 1
            
            # Ask if continue
            print("\n[INFO] Frame processing complete!")
            print("[INFO] Press Ctrl+C to exit, or wait for next frame...")
            
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[INFO] Closing serial connection...")
        ser.close()
        print("[SUCCESS] Serial port closed")
        print(f"[INFO] Total frames received: {frame_count}")
        print("=" * 50)

if __name__ == "__main__":
    main()

