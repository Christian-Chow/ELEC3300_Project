# STM32 Smart Home System (access control + internal environment monitoring)


this repo is the access control system code

Run the below code when STM32 is turned on and connected to the computer. Make sure the PORT and BAUD RATE are the same.


** Receiver side Python code **

```
import serial
import serial.tools.list_ports
import numpy as np
from PIL import Image
import time
import sys
import threading

# Configuration
SERIAL_PORT = 'COM3'  # Change to your port (e.g., '/dev/ttyUSB0' on Linux)
# BAUD_RATE = 921600
BAUD_RATE = 115200
FRAME_SIZE = 153600  # 320 * 240 * 2 bytes (RGB565)
WIDTH = 320
HEIGHT = 240

def input_handler(ser, stop_event):
    """Thread function to handle user input for sending data to STM32"""
    print("\n[INFO] Input handler started. Type messages to send to STM32.")
    print("[INFO] Type 'exit' or 'quit' to stop the program.\n")
    
    while not stop_event.is_set():
        try:
            # Get input from user (non-blocking would be better, but input() blocks)
            message = input("Send: ")
            
            # Check if user wants to exit
            if message.lower() in ['exit', 'quit']:
                stop_event.set()
                break
            
            # Send data to STM32
            if message:
                # Encode string to bytes and add newline character
                data = (message + '\n').encode('utf-8')
                ser.write(data)
                print(f"[SENT] {message}")
            else:
                # Send just a newline if empty input
                ser.write(b'\n')
                print("[SENT] (newline)")
        except EOFError:
            # Handle case when stdin is closed
            break
        except Exception as e:
            print(f"[ERROR] Error in input handler: {e}")

def main():
    print("=" * 50)
    print("STM32 Camera Frame Receiver with Send Capability")
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
    
    # Setup threading for input handling
    stop_event = threading.Event()
    input_thread = threading.Thread(target=input_handler, args=(ser, stop_event), daemon=True)
    input_thread.start()
    
    frame_count = 0
    
    try:
        while not stop_event.is_set():
            print("\n" + "-" * 50)
            print(f"[INFO] Waiting for frame data... (Frame #{frame_count + 1})")
            print("[INFO] You can send messages to STM32 while waiting (type in 'Send:' prompt)")
            
            # Wait for data - read in chunks to allow sending data
            start_time = time.time()
            data = bytearray()
            chunk_size = 4096  # Read in chunks to allow responsiveness
            
            print(f"[INFO] Reading {FRAME_SIZE} bytes from serial port...")
            while len(data) < FRAME_SIZE and not stop_event.is_set():
                # Check if there's data available
                if ser.in_waiting > 0:
                    # Read available data (up to chunk_size or remaining bytes needed)
                    remaining = FRAME_SIZE - len(data)
                    read_size = min(chunk_size, remaining, ser.in_waiting)
                    chunk = ser.read(read_size)
                    if chunk:
                        data.extend(chunk)
                        print(f"[DEBUG] Received {len(data)}/{FRAME_SIZE} bytes...", end='\r')
                else:
                    # Small delay to avoid busy waiting
                    time.sleep(0.01)
            
            if stop_event.is_set():
                break
                
            elapsed_time = time.time() - start_time
            data = bytes(data)
            
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
            print("[INFO] Waiting for next frame... (Type messages to send to STM32)")
            
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
        stop_event.set()
    except Exception as e:
        print(f"\n[ERROR] Unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        stop_event.set()
        print("\n[INFO] Closing serial connection...")
        if ser.is_open:
            ser.close()
        print("[SUCCESS] Serial port closed")
        print(f"[INFO] Total frames received: {frame_count}")
        print("=" * 50)

if __name__ == "__main__":
    main()



```
