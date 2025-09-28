import serial
import time
import numpy as np
from scipy.io.wavfile import write as write_wav
import sys

# --- Configuration (MUST match ESP32 C code settings) ---
SERIAL_PORT = '/dev/ttyACM0' # <-- CHANGE THIS to your ESP32's serial port (e.g., 'COM3' on Windows)
BAUD_RATE = 115200          # Matches the default ESP32 log/monitor speed
SAMPLE_RATE = 16000         # 16 kHz
CAPTURE_SECONDS = 5         # How many seconds of audio to capture
SAMPLE_WIDTH = 4            # Data width sent by ESP32 (32-bit integers)
NUM_CHANNELS = 1            # We configured I2S for MONO

# Calculate the total number of bytes we expect to read
TOTAL_BYTES_TO_READ = SAMPLE_RATE * NUM_CHANNELS * SAMPLE_WIDTH * CAPTURE_SECONDS

def capture_audio_to_wav(port, baud, duration, output_filename="captured_audio.wav"):
    """Reads raw audio data from serial and saves it as a 16-bit WAV file."""
    print(f"Connecting to serial port {port} at {baud}...")
    
    try:
        # Open the serial port
        ser = serial.Serial(port, baud, timeout=1)
        ser.flushInput() # Clear any old data in the buffer
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}. Please check the port name.")
        print(f"Details: {e}")
        return

    # --- START OF FIX: WAIT FOR AUDIO MARKER ---
    START_MARKER = b'START_AUDIO_DATA\n' # The marker sent by the C code
    END_MARKER = b'END_AUDIO_DATA\n'     # The new termination marker
    
    print("Waiting for audio data marker from ESP32 to start clean capture...")
    
    marker_found = False
    start_time = time.time() # Initialize start_time for timeout check here

    # Read line by line until the START marker is found, discarding all log messages
    while True:
        try:
            line = ser.readline()
            if not line:
                # If timeout occurs, break to avoid an infinite loop
                if time.time() - start_time > 20: 
                    print("\nError: Timed out waiting for start marker.")
                    break
                continue
                
            if START_MARKER.strip() in line:
                marker_found = True
                print("Marker received. Starting binary capture now.")
                break
                
            # print(f"Discarding: {line.decode().strip()}") # Uncomment for debug
            
        except serial.SerialTimeoutException:
            continue
            
    if not marker_found:
        ser.close()
        return
    # --- END OF FIX ---


    print(f"Capturing {duration} seconds of audio (Total bytes: {TOTAL_BYTES_TO_READ})...")
    
    # Read data in chunks
    audio_data = bytearray()
    start_time = time.time() # Reset start_time for the capture duration
    CHUNK_SIZE = 4096 # Read in 4KB chunks for efficiency
    
    while len(audio_data) < TOTAL_BYTES_TO_READ:
        # Calculate how many more bytes are needed (capped by CHUNK_SIZE)
        bytes_left_to_read = TOTAL_BYTES_TO_READ - len(audio_data)
        read_size = min(bytes_left_to_read, CHUNK_SIZE)

        # Read the data, waiting up to the serial timeout (1 second)
        data = ser.read(read_size) 
        
        if data:
            # 1. Check for the END marker within the received data chunk
            marker_index = data.find(END_MARKER)
            
            if marker_index != -1:
                # Marker found! Append only the data *before* the marker
                audio_data.extend(data[:marker_index])
                print(f"\nEND marker received. Capture stopped at {len(audio_data)} bytes.")
                break # Exit the capture loop early
            else:
                # 2. No marker, append all received data
                audio_data.extend(data)
        
        # 3. Check for external timeout (e.g., if ESP32 stopped sending data abruptly)
        elif time.time() - start_time > duration + 5: # Give 5 seconds grace period
            print("\nError: Timed out during audio data capture.")
            break
            
        # Give a status update
        elapsed = time.time() - start_time
        sys.stdout.write(f"\rProgress: {len(audio_data) / TOTAL_BYTES_TO_READ * 100:.2f}% | Time elapsed: {elapsed:.1f}s | Captured: {len(audio_data)} bytes")
        sys.stdout.flush()

    ser.close()
    print("\nCapture finished.")

    if not audio_data:
        print("Error: No data was received from the serial port.")
        return

    # --- Data Processing and Conversion ---
    
    # 1. Convert the raw bytes (32-bit signed integers) to a NumPy array
    # The ESP32 is little-endian, and the data is signed 32-bit integer ('<i4')
    raw_samples_32bit = np.frombuffer(audio_data, dtype='<i4')
    
    # 2. Convert 32-bit samples to standard 16-bit PCM for WAV
    # Safely shifting right by 8 bits converts 24-bit effective resolution to 16-bit.
    samples_16bit = (raw_samples_32bit >> 8).astype(np.int16)

    # 3. Save to WAV file
    try:
        write_wav(output_filename, SAMPLE_RATE, samples_16bit)
        print(f"Successfully saved audio to {output_filename}. Audio duration: {samples_16bit.size / SAMPLE_RATE:.2f}s")
    except Exception as e:
        print(f"Error saving WAV file: {e}")

if __name__ == "__main__":
    # Ensure you have 'pyserial', 'numpy', and 'scipy' installed:
    # pip install pyserial numpy scipy
    capture_audio_to_wav(SERIAL_PORT, BAUD_RATE, CAPTURE_SECONDS)
