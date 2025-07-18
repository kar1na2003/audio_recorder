# PSoC 6 Audio UART Receiver

This project captures audio data from a PSoC 6 device via UART and saves it as a WAV file on your PC.

---

## Usage Instructions

### Requirements

* Python 3.x
* `pyserial` library (`pip install pyserial`)

### Setup

1. Connect your PSoC 6 device UART output to your PC (e.g., via USB-UART converter).
2. Ensure your device is running the provided firmware that streams audio frames via UART.

### Running the Audio Receiver

```bash
python audio_receiver.py [COM_PORT]
```

* `COM_PORT` is optional (default is `COM9`).
* Baud rate is fixed at `460800`.
* Audio format: 16 kHz, mono, 16-bit samples.

### Commands

* `r` + Enter : Start recording audio to a WAV file.
* `s` + Enter : Stop recording.
* `q` + Enter : Quit the program.

### Output

* WAV files are saved as `audio_recording_YYYYMMDD_HHMMSS.wav` in the current directory.
* Audio frames are validated by start (`0xAA55`) and end (`0x55AA`) markers before writing.

---

### Example

```bash
python audio_receiver.py COM9
```

Type `r` to start recording, `s` to stop, and `q` to exit.

---

If you want, I can help you write full markdown with badges or installation sections!
