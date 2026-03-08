# SidecarTridge MV16 Emulator

The MV16 Emulator is a SidecarTridge Multi-device application for Atari ST/STE/Mega ST/Mega STE computers that emulates the MV16 audio device and routes its audio output through Bluetooth audio streaming. The MV-16 is a small cartridge that plugged into the expansion port of the Atari ST. It was included with the game B.A.T. and included the ability to playback sampled sounds with up to 16 voices at a time. The MV-16's use was not limited to B.A.T. and other programs could take advantage of its sound capabilities as well.

## 🚀 Installation

To install the MV16 Emulator app on your SidecarTridge Multi-device:

1. Launch the **Booster App** on your SidecarTridge Multi-device.
2. Open the Booster web interface.
3. In the **Apps** tab, select **"MV16 Emulator"** from the list of available apps.
4. Click **"Download"** to install the app to your SidecarTridge’s microSD card.
5. Once installed, select the app and click **"Launch"** to activate it.

After launching, the app will automatically run every time your Atari computer is powered on.

## 🕹️ Usage

When the application starts, it opens a simple terminal menu on the Atari ST.

From there, the default action is to start scanning for nearby Bluetooth audio devices. Once a device is found and selected, the firmware connects to it and starts the A2DP audio path.

After the connection is established, the user can:

- return to the desktop while keeping the Bluetooth side active
- scan again for another device
- jump to the Booster application
- exit the application completely

The idea is to keep the user flow simple: start the app, connect audio, and continue using the computer.

### ⚙️ Setup Screen Commands
| Command | Description |
|---------|-------------|
| **[S]can** | Start scanning for nearby Bluetooth audio devices. |
| **[E]xit to Desktop** | Exit to desktop without loading the ROM. |
| **[X] Return to the Booster menu** | Exit setup and return to the Booster Loader main menu. |

### 🔁 System Reset Behavior

The app is **resistant to system resets**. Pressing the reset button on your Atari will not cause the app to exit or lose its state. The app will continue running and maintain its connection to the Bluetooth audio device, allowing you to reset your Atari without interrupting your audio experience.

### 🔌 Power Cycling

When you power off and on your Atari, the app will automatically start again and attempt to reconnect to the last paired Bluetooth audio device. This means you can power cycle your Atari without needing to manually restart the app or reselect your audio device.

### ❌ SELECT Button Behavior
Pressing the SELECT button on your SidecarTridge Multi-device will cause the app to return to the app main menu. This allows you to easily exit the app and return to the Booster Loader main menu if needed.

## 🛠️ Setting Up the Development Environment

This project is based on the [SidecarTridge Multi-device Microfirmware App Template](https://github.com/sidecartridge/md-microfirmware-template).  
To set up your development environment, please follow the instructions provided in the [official documentation](https://docs.sidecartridge.com/sidecartridge-multidevice/programming/).

## 📄 License

This project is licensed under the **GNU General Public License v3.0**.  
See the [LICENSE](https://github.com/sidecartridge/md-rom-emulator/blob/main/LICENSE) file for full terms.

## 🤝 Contributing
Made with ❤️ by [SidecarTridge](https://sidecartridge.com)
