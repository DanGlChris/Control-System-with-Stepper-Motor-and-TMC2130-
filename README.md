# High-Precision Control System for Industrial Manipulators

This repository contains code and documentation for a high-precision control system designed for industrial manipulators, such as robotic arms or CNC machines. The system uses a NEMA 17 stepper motor, the TMC2130 driver, and an Arduino Pro Micro microcontroller to deliver accurate, adaptive, and energy-efficient motion control.

## Repository Contents

- **Arduino Code (C++)**:  
  Contains the main firmware (`Perfect_work.ino`) for the Arduino Pro Micro. This code initializes and configures the TMC2130 stepper motor driver (using the TMC2130 library), sets up microstepping, current control, and handles the precise movement of the NEMA 17 stepper motor.

- **HMI (Python with Qt)**:  
  The repository also includes a Python-based Human-Machine Interface (HMI) application built with Qt. This HMI allows users to interact with and monitor the control system, adjust parameters, and visualize the system's performance.

- **Documentation**:  
  The PDF file (`Control System with stepper motor and TMC2130.pdf`) provides detailed information on the system's architecture, component selection, driver configuration, programming, and testing.

## Usage

1. **Hardware Setup**:
   - Connect the Arduino Pro Micro, TMC2130 driver, and NEMA 17 stepper motor as outlined in the documentation.

2. **Software Installation**:

   - **For Arduino Code**:
     - Install the [Arduino IDE](https://www.arduino.cc/en/software) (version 1.8.13 or later).
     - Install the TMC2130Stepper library (version 2.0.0 or later) using the Arduino IDE's Library Manager.
     - Open `Perfect_work.ino` in the Arduino IDE and upload it to the Arduino Pro Micro.

   - **For the HMI Application (Python with Qt)**:
     - Ensure Python (3.6 or later) is installed on your system.
     - Install PyQt5 or the appropriate Qt library for Python. This can typically be done via pip:
       ```bash
       pip install PyQt5
       ```
     - Run the HMI application script (e.g., `Dashboard.py`) to start the interface.

3. **Configuration**:
   - Refer to the `Control System with stepper motor and TMC2130.pdf` for detailed instructions on configuring the TMC2130 driver's registers and the HMI settings tailored to your specific hardware configuration.

4. **Testing**:
   - For the firmware: Run the test routines embedded in the Arduino code to verify motor control performance.
   - For the HMI: Use the interface to test real-time communication and parameter adjustments with the control system.

## Prerequisites

- Arduino Pro Micro (or compatible microcontroller)
- TMC2130 stepper motor driver and its corresponding library [(TMC2130Stepper)](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2130_datasheet_rev1.15.pdf)
- NEMA 17 stepper motor
- 12V power supply
- Arduino IDE (version 1.8.13 or later)
- Python (3.9 or later)
- Qt for Python (e.g., PyQt5)

## Contributing

Contributions are welcome! Feel free to submit a pull request or open an issue to discuss improvements or bug fixes.

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For questions or issues, please open an issue on this repository.
