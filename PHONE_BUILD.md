# Build this ESP32-CAM firmware using only an Android phone

This project intentionally uses native ESP-IDF because the required HFP Voice-over-HCI software-audio path is not a reliable Arduino-only feature on the original ESP32.

## 1. Create a GitHub repository

On your Android phone, open GitHub in your browser and create a new empty repository.

## 2. Upload this project's files

Upload the contents of this folder so that `.github/workflows/build.yml`, `main/main.cpp`, `main/CMakeLists.txt`, `main/idf_component.yml`, `CMakeLists.txt`, and `sdkconfig.defaults` are at the repository root.

## 3. Start the build

Open the repository's **Actions** tab.
Choose **Build ESP32-CAM AI firmware**.
Press **Run workflow**.

GitHub will compile the firmware in an ESP-IDF 5.5.2 environment. No PC is required.

## 4. Download the firmware

When the workflow finishes, open the completed run and download the artifact named:

`esp32-cam-ai-firmware`

It contains:

- `bootloader.bin`
- `partitions.bin`
- `ai-cam.bin`
- `FLASH_OFFSETS.txt`

## 5. Flash with your Android ESP flasher

Use these offsets exactly:

- `bootloader.bin` → `0x1000`
- `partitions.bin` → `0x8000`
- `ai-cam.bin` → `0x10000`

Select the ESP32-CAM / ESP32 target and flash through the ESP32-CAM-MB USB programmer.

## Important

Before building, edit `main/main.cpp` and replace the four configuration values:

- `WIFI_SSID`
- `WIFI_PASSWORD`
- `VISION_API_KEY`
- `SARVAM_API_KEY`

The firmware is written for the original AI-Thinker ESP32-CAM (ESP32, not ESP32-S2/S3/C3).
