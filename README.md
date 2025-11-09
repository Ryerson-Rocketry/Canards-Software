<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![project_license][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/Ryerson-Rocketry/Canards-Software">
    <img src="MetRocketry_Logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Canards Software</h3>

  <p align="center">
    The firmware code for the Canards
    <br />
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
This repository contains the complete avionics firmware for Canards, developed by the Control Systems subteam for the MetRocketry student design team. This high-performance flight control system is built on the STM32F446RE microcontroller and is designed to provide active canard stabilization and real-time trajectory correction for high-powered rocketry, running on a lightweight FreeRTOS kernel for deterministic, multi-threaded task management.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* ![STM32](https://img.shields.io/badge/MCU-STM32-blue?logo=STMicroelectronics&logoColor=white)  
* ![C](https://img.shields.io/badge/Language-C-blue?logo=c&logoColor=white)
* ![C++](https://img.shields.io/badge/Language-C++-00599C?logo=c%2B%2B&logoColor=white)
* ![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-5ebc28?logo=freertos&logoColor=white)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started
<!-- TODO: Write how to run this project -->
This project is set up for an easy, one-click workflow.

1. **Install Prerequisites**

   * Download and install all the software listed in the "Prerequisites" section below.

2. **Set Up Your Path**

   * **This is a crucial step!** You must add the `bin` folders for `arm-none-eabi-gcc` and `openocd` to your system's Environment Variables (PATH).

   * **Verify:** You can check this by opening a **new** terminal and typing `arm-none-eabi-gcc --version` and `openocd --version`. You should see version info, not a "command not found" error.

3. **Build & Debug**

   * **VS Code (Recommended):** Open this folder in VS Code, connect your ST-Link, and press **F5**. The `preLaunchTask` will automatically run the `build.sh` script and start the debugger.

   * **Manual (Terminal):**

     * `./build.sh Debug` (Builds the debug `.elf` file)

     * `./build.sh Release` (Builds the optimized release `.elf` file)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Prerequisites
To build and debug this project, you need the following software installed and added to your system's `PATH`.

1.  **ARM Toolchain (`arm-none-eabi-gcc`)**
    * **What it is:** Provides the `gcc` compiler, `gdb` debugger, and other tools to build for an ARM chip.
    * **Where to get it:** [ARM GNU Toolchain Downloads](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

2.  **OpenOCD (Open On-Chip Debugger)**
    * **What it is:** The GDB server software that connects to your ST-Link hardware probe.
    * **Where to get it:** [xPack OpenOCD Releases](https://github.com/xpack-dev-tools/openocd-xpack/releases) (Recommended to get the pre-compiled binaries, *not* the source code).

3.  **VS Code + Cortex-Debug Extension**
    * **What it is:** Your code editor and the extension that understands how to talk to OpenOCD and GDB.
    * **Where to get it:**
        * [Visual Studio Code](https://code.visualstudio.com/)
        * [Cortex-Debug Extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)

4.  **Bash Shell** (For `build.sh`)
    * **Windows:** Automatically included with [Git for Windows](https://git-scm.com/install/windows) (as "Git Bash").
    * **macOS / Linux:** Included by default.
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap
<!-- TODO: Come up with a road map for this project -->
WIP

<!-- CONTRIBUTING -->
## Contributing

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Top contributors:

<a href="https://github.com/Ryerson-Rocketry/Canards-Software/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Ryerson-Rocketry/Canards-Software" alt="contrib.rocks image" />
</a>

<!-- CONTACT -->
## Contact
Contact MetRocketry: [https://www.metrocketry.com/](https://www.metrocketry.com/) </br>
Project Link: [https://github.com/Ryerson-Rocketry/Canards-Software](https://github.com/Ryerson-Rocketry/Canards-Software)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Madgwick, S. O. H. "An efficient orientation filter for inertial and inertial/magnetic sensor arrays." (2010)](https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf)  
    - The original Madgwick filter algorithm paper, for cool demonstration purposes 
* [othneildrew/Best-README-Template](https://github.com/othneildrew/Best-README-Template)  
    - For the README template.
    
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[contributors-url]: https://github.com/Ryerson-Rocketry/Canards-Software/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[forks-url]: https://github.com/Ryerson-Rocketry/Canards-Software/network/members
[stars-shield]: https://img.shields.io/github/stars/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[stars-url]: https://github.com/Ryerson-Rocketry/Canards-Softwarestargazers
[issues-shield]: https://img.shields.io/github/issues/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[issues-url]: https://github.com/Ryerson-Rocketry/Canards-Software/issues
[license-shield]: https://img.shields.io/github/license/Ryerson-Rocketry/Canards-Software.svg?style=for-the-badge
[license-url]: https://github.com/Ryerson-Rocketry/Canards-Software/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/metrocketry
