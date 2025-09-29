import serial
import wave

PORT = '/dev/ttyUSB0'  # Change to your serial port, e.g. COM3 on Windows
BAUD_RATE = 115200
OUTPUT_FILE = 'recorded_audio.wav'
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16 bits = 2 bytes

def main():
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"Listening on {PORT} at {BAUD_RATE} baud... Press Ctrl+C to stop and save.")

    wf = wave.open(OUTPUT_FILE, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(SAMPLE_WIDTH)
    wf.setframerate(SAMPLE_RATE)

    try:
        while True:
            data = ser.read(1024)
            if data:
                wf.writeframes(data)
    except KeyboardInterrupt:
        print("\nStopping and saving file.")
    finally:
        wf.close()
        ser.close()
        print(f"Saved audio to {OUTPUT_FILE}")

if __name__ == '__main__':
    main()
