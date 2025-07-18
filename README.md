Sure! Here is the full text of the README.md file:

---

# PSoC 6 UART Audio Streaming over USB-UART

This repository contains:

* **PSoC 6 firmware** (using HAL APIs) that captures audio from a PDM microphone and transmits PCM audio data over UART via USB (virtual COM port).
* **Python script** to receive audio data over the USB virtual COM port and save it as a WAV file on a PC.

---

## Features

* Real-time audio capture from PDM microphone on PSoC 6.
* Audio data streamed over UART with start/end frame markers for integrity.
* USB-UART bridge or virtual COM port support for easy PC connection.
* Python script for receiving, validating, and saving audio to WAV format.
* Simple console commands to start/stop recording and quit.

---

## Requirements

* **ModusToolbox** for building and flashing PSoC 6 firmware.
* **Python 3.x** on PC.
* Python package: [`pyserial`](https://pypi.org/project/pyserial/).

Install `pyserial` with:

```bash
pip install pyserial
```

---

## Setup and Usage

### 1. Flash PSoC 6 Firmware

* Open the project in ModusToolbox.
* Configure PDM/PCM audio input pins and UART pins.
* Build and flash the firmware to your PSoC 6 device.
* The firmware continuously captures audio and sends it over UART.

### 2. Connect Hardware

* Connect your PSoC 6 board to the PC via USB.
* The device will enumerate as a **USB virtual COM port**.
* Note the COM port assigned (e.g., `COM9` on Windows, `/dev/ttyUSB0` on Linux).

### 3. Run Python Audio Receiver

Run the Python script with the COM port:

```bash
python audio_receiver.py COM9
```

If you do not specify a port, it defaults to `COM9`.

---

## Python Script Commands

| Command | Description                  |
| ------- | ---------------------------- |
| `r`     | Start recording audio to WAV |
| `s`     | Stop recording               |
| `q`     | Quit the program             |

---

## Output

* Recorded WAV files are saved as `audio_recording_YYYYMMDD_HHMMSS.wav` in the current directory.
* Audio data frames are validated by start marker (`0xAA55`) and end marker (`0x55AA`).

---

## Notes

* UART communication is over USB using a USB-UART bridge or built-in virtual COM port.
* Ensure common ground between the PSoC 6 device and USB-UART converter.
* Baud rate is fixed at 460800 baud.
* Audio parameters: 16 kHz sample rate, mono channel, 16-bit samples.

---

## License & Disclaimer

This software includes code provided by Cypress Semiconductor Corporation (an Infineon company) under their licensing terms. Please see source file headers for details.

---
