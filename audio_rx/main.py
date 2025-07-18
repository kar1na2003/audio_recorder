#!/usr/bin/env python3

import serial
import struct
import wave
import time
import sys
import threading
from datetime import datetime

# UART and audio settings
UART_PORT = 'COM9'
BAUD_RATE = 460800
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2
FRAME_SIZE = 512

# Frame markers
AUDIO_START_MARKER = 0xAA55
AUDIO_END_MARKER = 0x55AA

class AudioReceiver:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.is_recording = False
        self.wav_file = None
        self.audio_data = []
        self.running = False

    def connect(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1,
                rtscts=True
            )
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected from UART")

    def create_wav_file(self, filename):
        try:
            self.wav_file = wave.open(filename, 'wb')
            self.wav_file.setnchannels(CHANNELS)
            self.wav_file.setsampwidth(SAMPLE_WIDTH)
            self.wav_file.setframerate(SAMPLE_RATE)
            print(f"Created WAV file: {filename}")
            return True
        except Exception as e:
            print(f"Error creating WAV file: {e}")
            return False

    def close_wav_file(self):
        if self.wav_file:
            self.wav_file.close()
            self.wav_file = None
            print("WAV file closed")

    def read_uart_data(self):
        buffer = bytearray()

        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(data)

                    # Process complete audio frames
                    while len(buffer) >= 4:
                        start_idx = -1
                        for i in range(len(buffer) - 1):
                            marker = struct.unpack('<H', buffer[i:i+2])[0]
                            if marker == AUDIO_START_MARKER:
                                start_idx = i
                                break

                        if start_idx == -1:
                            if len(buffer) > 1:
                                buffer = buffer[-1:]
                            break

                        if start_idx > 0:
                            buffer = buffer[start_idx:]

                        expected_frame_size = 2 + (FRAME_SIZE * 2) + 2  # start + audio + end
                        if len(buffer) < expected_frame_size:
                            break

                        frame_data = buffer[2:2 + (FRAME_SIZE * 2)]
                        end_marker_pos = 2 + (FRAME_SIZE * 2)

                        if len(buffer) >= end_marker_pos + 2:
                            end_marker = struct.unpack('<H', buffer[end_marker_pos:end_marker_pos+2])[0]

                            if end_marker == AUDIO_END_MARKER:
                                # Valid frame, write to WAV
                                if self.is_recording and self.wav_file:
                                    self.wav_file.writeframes(frame_data)
                                buffer = buffer[end_marker_pos + 2:]

                                if self.is_recording:
                                    print(f"Recording... Frame received ({len(frame_data)} bytes)")
                            else:
                                buffer = buffer[2:]
                        else:
                            break
                else:
                    time.sleep(0.001)

            except Exception as e:
                print(f"Error reading UART data: {e}")
                break

    def start_recording(self, filename=None):
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"audio_recording_{timestamp}.wav"

        if self.create_wav_file(filename):
            self.is_recording = True
            print("Recording started. Press Enter to stop...")
            return True
        return False

    def stop_recording(self):
        self.is_recording = False
        self.close_wav_file()
        print("Recording stopped")

    def run(self):
        if not self.connect():
            return

        self.running = True

        # Start UART reading in a background thread
        uart_thread = threading.Thread(target=self.read_uart_data)
        uart_thread.daemon = True
        uart_thread.start()

        try:
            print("\nAudio Receiver Ready!")
            print("Commands:")
            print("  'r' + Enter: Start recording")
            print("  's' + Enter: Stop recording")
            print("  'q' + Enter: Quit")
            print("\nWaiting for commands...")

            while self.running:
                try:
                    cmd = input().strip().lower()

                    if cmd == 'r':
                        if not self.is_recording:
                            self.start_recording()
                        else:
                            print("Already recording!")

                    elif cmd == 's':
                        if self.is_recording:
                            self.stop_recording()
                        else:
                            print("Not recording!")

                    elif cmd == 'q':
                        print("Exiting...")
                        break

                    else:
                        print("Unknown command. Use 'r' to record, 's' to stop, 'q' to quit.")

                except KeyboardInterrupt:
                    print("\nExiting...")
                    break
                except EOFError:
                    break

        finally:
            self.running = False
            if self.is_recording:
                self.stop_recording()
            self.disconnect()

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else UART_PORT

    print(f"PSoC 6 Audio Receiver")
    print(f"Port: {port}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Sample Rate: {SAMPLE_RATE} Hz")
    print(f"Format: {CHANNELS} channel, {SAMPLE_WIDTH*8}-bit")
    print("-" * 50)

    receiver = AudioReceiver(port, BAUD_RATE)
    receiver.run()

if __name__ == "__main__":
    main()
