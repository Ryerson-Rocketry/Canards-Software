<a id="readme-top"></a>
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![project_license][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

---

<br />
<div align="center">
  <a href="https://github.com/Ryerson-Rocketry/Canards-Software">
    <img src="MetRocketry_Logo.png" alt="Logo" width="180" height="180">
  </a>

<h3 align="center">Canards Avionics Firmware</h3>

  <p align="center">
    **High-Performance Active Stabilization for High-Powered Launch Vehicles.**
    <br />
    The flight control system built on STM32F446RE and FreeRTOS.
    <br />
    <a href="https://github.com/Ryerson-Rocketry/Canards-Software/issues">Report Bug</a>
    ·
    <a href="https://github.com/Ryerson-Rocketry/Canards-Software/issues">Request Feature</a>
  </p>
</div>

---

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#key-features">Key Features</a></li>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#build--debug-steps">Build & Debug Steps</a></li>
      </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

---

## 🚀 About The Project

This repository contains the complete avionics firmware for the **Canards** flight vehicle, developed by the Control Systems subteam of MetRocketry student design team.

The system is engineered for reliable, high-speed control, utilizing the computational power of the **STM32F446RE** microcontroller. It is designed to provide active canard stabilization and real-time trajectory correction, ensuring mission success in high-powered rocketry flights.

### ✨ Key Features

| Feature | Description |
| :--- | :--- |
| **Complete 3-Axis Stabilization** | Provides stabilization across **Roll** axes for comprehensive flight control and disturbance rejection. |
| **Active Roll Control** | Implements high-frequency control to maintain a precise target roll angle (typically $0^\circ$), actively mitigating roll-induced coupling. |
| **High-Fidelity PID Control** | Utilizes a robust **Proportional-Integral-Derivative (PID) control algorithm** for closed-loop stability, guaranteeing rapid response and minimal steady-state error. |
| **Deterministic RTOS** | Built on a lightweight **FreeRTOS** kernel, ensuring deterministic and reliable execution of time-critical tasks (sensor fusion, state estimation, and control). |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### 🛠️ Built With

| Component | Description | Technologies |
| :--- | :--- | :--- |
| **Microcontroller** | Core flight control processor. | **STM32F446RE** (Cortex-M4) <br> ![STM32](https://img.shields.io/badge/MCU-STM32-blue?logo=STMicroelectronics&logoColor=white) |
| **RTOS** | Provides deterministic multi-threaded task management. | **FreeRTOS** <br> ![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-5ebc28?logo=freertos&logoColor=white) |
| **Languages** | Primary development languages. | **C / C++** <br> ![C](https://img.shields.io/badge/Language-C-blue?logo=c&logoColor=white)   ![C++](https://img.shields.io/badge/Language-C++-00599C?logo=c%2B%2B&logoColor=white) |
| **Toolchain** | Compiler and build system. | GCC/Make-based system. |
| **Configuration** | Hardware initialization setup. | STM32CubeMX (`.ioc` file). |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🛠️ Getting Started

This section outlines the setup required to successfully build, flash, and debug the firmware.

### 🌱 Prerequisites

To build and debug this project, you need the following software installed and their executable paths configured in your system's **PATH** environment variable.

1.  **ARM GNU Toolchain** (`arm-none-eabi-gcc`)
    * **Purpose:** The cross-compiler and linker suite for the ARM architecture.
    * **Download:** [ARM GNU Toolchain Downloads](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-chain/downloads)

2.  **OpenOCD** (Open On-Chip Debugger)
    * **Purpose:** The GDB server software that bridges the debugger with the hardware probe (e.g., ST-Link).
    * **Download:** [xPack OpenOCD Releases](https://github.com/xpack-dev-tools/openocd-xpack/releases)

3.  **VS Code + Cortex-Debug Extension**
    * **Purpose:** Recommended IDE and debugging extension for embedded development.

4.  **Bash Shell**
    * **Purpose:** Required to execute the `./build.sh` and `./run.sh` scripts.

### 🫡 Build & Debug Steps

This project is set up for an easy, one-click workflow within VS Code.

1.  **Verify PATH:** Open a terminal and ensure the toolchain is accessible:
    ```bash
    arm-none-eabi-gcc --version
    openocd --version
    ```
2.  **Connect Hardware:** Plug in your **ST-Link/J-Link** probe to the development board.
3.  **Docker USB Passthrough (Optional):** If developing inside a Docker container, you must expose the host's USB bus to the container so OpenOCD can see the ST-Link hardware. Run your container with the following flags:
    ```bash
    # Mount the USB bus and run in privileged mode
    docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb <image_name_or_id>
    ```
4.  **VS Code (Recommended):**
    * Open this repository folder in VS Code.
    * Go to the Run and Debug view (F5).
    * Press **F5**. The configured `preLaunchTask` will automatically run `./build.sh Debug` and launch the debugger.
5.  **Manual (Terminal):**
    * To build the final binary: `./build.sh Release`
    * To build and enable debugging symbols: `./build.sh Debug`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🗺️ Roadmap
* Implement Kalman filter for altitude and velocity determination.
* Integrate data logging to external SD card.
* Develop robust flight simulation and hardware-in-the-loop (HIL) testing environment.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🤝 Contributing

Contributions make the rocketry world go 'round! If you have suggestions or find bugs, please fork the repo and create a pull request.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'feat: Add Amazing Feature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### 💖 Top Contributors:
<a href="https://github.com/Ryerson-Rocketry/Canards-Software/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Ryerson-Rocketry/Canards-Software" alt="contrib.rocks image" />
</a>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 📧 Contact

MetRocketry Team Website: [https://www.metrocketry.com/](https://www.metrocketry.com/) <br/>
Project Repository: [https://github.com/Ryerson-Rocketry/Canards-Software](https://github.com/Ryerson-Rocketry/Canards-Software)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## 🏆 Acknowledgments

* [**othneildrew/Best-README-Template**](https://github.com/othneildrew/Best-README-Template) <br/>
    *For providing the clean and structured Markdown template.*
    
<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

[contributors-shield]: https://img.shields.io/github/contributors/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[contributors-url]: https://github.com/Ryerson-Rocketry/Canards-Software/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[forks-url]: https://github.com/Ryerson-Rocketry/Canards-Software/network/members
[stars-shield]: https://img.shields.io/github/stars/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[stars-url]: https://github.com/Ryerson-Rocketry/Canards-Softwarestargazers
[issues-shield]: https://img.shields.io/github/issues/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[issues-url]: https://github.com/Ryerson-Rocketry/Canards-Software/issues
[license-shield]: https://img.shields.io/badge/license-Unlicensed-red?style=for-the-badge&logo=gitea
[license-url]: https://github.com/Ryerson-Rocketry/Canards-Software/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/LinkedIn-blue.svg?style=for-the-badge&logo=linkedin&logoColor=white
[linkedin-url]: https://www.linkedin.com/company/metrocketry/