# ESP32-S3 Multi-Node Communication Project

This project uses ESP32-S3 boards for a multi-node communication system, leveraging ESP-IDF, RTOS, and ESP-NOW for wireless communication between master and slave nodes.

## Table of Contents

1. [Description](#description)  
2. [Project Structure](#project-structure)  
3. [Features](#features)  
4. [Setup](#setup)  
5. [Usage](#usage)  
6. [Acknowledgments](#acknowledgments)  

---

## Description

This project demonstrates a communication network using multiple ESP32-S3 boards. The master node collects data from the slave nodes and controls devices such as:

1. **Fan control** based on temperature readings.  
2. **Light control** based on motion detection.  
3. **Curtain control** based on environmental conditions (e.g., light levels or time).  

The communication between nodes uses **ESP-NOW**, and the system runs on **RTOS (Real-Time Operating System)** for efficient task management.

---

## Project Structure

```plaintext
project-root/
│-- main/
│   ├── master.c          # Code for the master node
│   ├── fanSlave.c        # Slave node controlling the fan
│   ├── lightSlave.c      # Slave node controlling lights
│   └── curtainSlave.c    # Slave node controlling curtains
│
│-- managed_components/   # Custom components
│
│-- CMakeLists.txt        # Project build configuration
│-- README.md             # Project documentation (this file)
└-- sdkconfig             # ESP-IDF project configuration

```

## Features

- **ESP-NOW Communication**: Low-latency, connectionless communication between nodes.
- **RTOS Integration**: Multi-tasking and efficient resource management.
- **Fan Control**: Activates based on temperature thresholds.
- **Light Control**: Activates based on motion detection.
- **Curtain Control**: Opens/closes curtains based on environmental conditions.
- **Scalability**: Easy to add more slave nodes with different functionalities.


## Setup

### Prerequisites

- ESP-IDF installed (Installation Guide)
- Visual Studio Code with the ESP-IDF Extension
- ESP32-S3 Boards

### Installation Steps

1. **Clone the repository**:
    ```bash
    git clone https://github.com/Cynthilizy/ESP-NOW_Project.git
    cd ESP-NOW_Project
    ```

2. **Set up ESP-IDF environment**:
    ```bash
    . $HOME/esp/esp-idf/export.sh
    ```

3. **Configure the target**:
    ```bash
    idf.py set-target esp32s3
    ```

4. **Build the project**:
    ```bash
    idf.py build
    ```

5. **Flash the firmware to your ESP32-S3 board**:
    ```bash
    idf.py flash
    ```

6. **Monitor the output**:
    ```bash
    idf.py monitor
    ```

## Usage

1. Connect the master node to your computer and flash the `master.c` firmware.
2. Connect each slave node and flash the corresponding firmware:
    - `fanSlave.c` for fan control.
    - `lightSlave.c` for light control.
    - `curtainSlave.c` for curtain control.
3. Power all nodes and monitor communication via the serial console.

*Note: It might be easier to separate each node into individual folders before building and flashing.*

## Acknowledgments

- Espressif for the ESP-IDF framework.
- Community Resources for ESP32 and ESP-NOW examples.
```