# ESP32-CAM AI Vision + Bluetooth HFP

This is the phone-buildable version of the project.

Hardware target: AI-Thinker ESP32-CAM (original ESP32 with Classic Bluetooth) + Realme Buds Wireless 3 Neo.

Flow:

Realme neckband voice button -> HFP voice-recognition event -> neckband microphone -> ESP32 -> Sarvam STT -> camera capture -> CircuitDigest Vision AI -> Sarvam TTS -> HFP -> neckband speaker.

The actual Bluetooth audio path uses native ESP-IDF HFP Audio Gateway Voice-over-HCI, not BluetoothSerial/SPP.

Use `PHONE_BUILD.md` for Android-only compilation via GitHub Actions.
