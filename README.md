# Sensors Platform (SP) v1 in Field4D (F4D) Outdoor Plant Phenotyping iIoT Framework

| ![Field4D Logo](https://avatars.githubusercontent.com/u/71197432?v=4) | ![Yellow Ribbon](https://upload.wikimedia.org/wikipedia/commons/thumb/d/d0/Yellow_ribbon.svg/200px-Yellow_ribbon.svg.png) |
|:----------------------:|:----------------------:|

## Description

[F4D (Field4D) outdoor plant phenotyping iIoT framework](https://github.com/field4d/field4d): Sensors Platform (SP) firmware source code.

---

## Overview

The **Sensors Platform (SP)** is a **low-power child node** that functions as an **environmental data acquisition unit** within the **Field4D (F4D) Outdoor Plant Phenotyping** ecosystem. It collects **sensor readings** such as **temperature, humidity and light intensity**, transmitting them to the **Base Edge Router (BER)** over **UDP using RPL-TSCH**.

## Hardware Implementation

- **TI CC2650 SensorTag (CC2650STK):** A compact wireless MCU platform featuring a 32-bit ARM® Cortex®-M3 processor at 48 MHz, optimized for **low-power** sensor data acquisition and wireless communication in outdoor phenotyping applications.

## SimpleLink Software Environment

The SimpleLink software environment offers modular **wireless connectivity**, TI Drivers, NoRTOS support, and a Core SDK provided as a **git submodule**. It supports **UART, SPI, I2C**, and other peripherals, with configurable options in `project-conf.h` for memory and code optimization.

## Usage

This firmware is tailored for `TARGET=cc26x0-cc13x0 BOARD=sensortag/cc2650`.

### Prerequisites

Install all required toolchains as per the Contiki-NG documentation.

### Building the Firmware

In the project root, execute:

```shell
make distclean
make TARGET=cc26x0-cc13x0 BOARD=sensortag/cc2650
```

## License

MIT License

## Contributors

- **Idan Ifrach**, Hebrew University of Jerusalem, Israel
  [idan.ifrach@mail.huji.ac.il](mailto:idan.ifrach@mail.huji.ac.il) | [LinkedIn](https://www.linkedin.com/in/ifrachi/) | [ORCID](https://orcid.org/0009-0000-0552-0935)

## References

![Logo](https://raw.githubusercontent.com/contiki-ng/contiki-ng.github.io/master/images/logo/Contiki_logo_1RGB.png)

---

**Contiki-NG: The OS for Next Generation IoT Devices**
Contiki-NG is an **open-source OS for IoT devices**, focusing on **secure, reliable low-power communication** with **IPv6/6LoWPAN, 6TiSCH, RPL, and CoAP**.

- [GitHub repository](https://github.com/contiki-ng/contiki-ng)
- [Documentation](https://docs.contiki-ng.org/)
- [Releases](https://github.com/contiki-ng/contiki-ng/releases)
- [Website](http://contiki-ng.org)

## Contact

For support, contact <greenroom.lab@mail.huji.ac.il>
