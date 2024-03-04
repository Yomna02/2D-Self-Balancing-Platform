# 2D-Self-Balancing-Platform

## Overview
This project aims to achieve precise control using PD controller over the orientation of a platform holder using an Arduino Nano microcontroller, an IMU sensor, and two servo motors. The entire system is programmed using LabVIEW, providing a user-friendly interface for control and monitoring. This README provides an overview of the project, details the hardware assembly and components used, describes the LabVIEW interface, explains the controller design, and outlines the control implementation process.

## Hardware Assembly and Components
The following components were used to build the project:
- Arduino Nano Microcontroller
- MPU 6050 Sensor
- 2 Hitec HS-422 Standard Deluxe Servo Motors
- Mini Breadboard
- 2 DOF Aluminum Platform

![Connections](https://github.com/Yomna02/2D-Self-Balancing-Platform/blob/main/Media/Wiring.png?raw=true)
<img src="https://github.com/Yomna02/2D-Self-Balancing-Platform/blob/main/Media/Wiring.png?raw=true" alt="Connections" width="500" height="400">
![Final Assembled Setup](https://github.com/Yomna02/2D-Self-Balancing-Platform/blob/main/Media/Isometric%20View.jpg?raw=true)
<img src="https://github.com/Yomna02/2D-Self-Balancing-Platform/blob/main/Media/Isometric%20View.jpg?raw=true" alt="Final Assembled Setup" width="500" height="400">

The circuit wiring and hardware setup can be found in the project documentation, including figures illustrating the connections and the final assembled setup.

## LabVIEW Interface
LabVIEW serves as the control interface tool for this project, providing a user-friendly environment for monitoring and controlling the platform. The LabVIEW graphical programming interface and LINX toolbox are utilized to input desired parameters, view real-time position feedback, and adjust control settings. The front control panel in LabVIEW displays all the relevant parameters and system state indicators.

![Front Panel](https://github.com/Yomna02/2D-Self-Balancing-Platform/blob/main/Media/Front%20Panel.png?raw=true)
<img src="https://github.com/Yomna02/2D-Self-Balancing-Platform/blob/main/Media/Front%20Panel.png?raw=true" alt="Front Panel" width="500" height="400">

## Controller Design
The control system incorporates a PID controller to achieve the desired orientation of the platform holder. The PID controller takes a desired setpoint and a feedback signal as inputs and generates an actuation signal for the servo motors for roll and pitch directions. The gains for the proportional (P) and proportional-derivative (PD) components of the controller were fine-tuned to achieve optimal performance, balancing responsiveness, accuracy, and smoothness of movement.

<img src="https://github.com/Yomna02/2D-Self-Balancing-Platform/blob/main/Media/PID%20Controller.png?raw=true" alt="PID Controller" width="500" height="400">

## Control Implementation: Sensor Calibration, Servo Write, Integration with Controller
The control implementation involves several steps, including sensor calibration, servo motor mapping, and integration with the PID controller. 

**Sensor Calibration:** The IMU sensor (MPU-6050) is calibrated to obtain smooth and accurate signals for roll and pitch. The accelerometer readings are used to compute the roll and pitch angles directly, while the gyroscope readings are integrated to obtain the angular orientations. The sensor data from both sources is then processed using a complementary filter with weights to combine them into filtered outputs for each axis.

**Servo Write:** The servo motors (Hitec HS-422) are mapped to convert the desired angle inputs into the appropriate PWM signals. The PWM range and the neutral position of the servos are adjusted for symmetry. The servo write sub-VI implements the mapping equations and sends the PWM signals to the Arduino Nano for controlling the servo motors.

**Integration with Controller:** The sensor data and the desired setpoint are integrated into a LabVIEW VI file, which initializes the Arduino Nano, reads data from the IMU sensor, and sends commands to the PID controller. The PID controller processes the error signal and generates the actuation signal for the servo motors. The system's orientation is continuously monitored and displayed on the LabVIEW interface.

## Final Demonstration
The integrated system demonstrates precise control over the orientation of the platform holder. The LabVIEW interface allows users to input desired parameters, monitor real-time position feedback, and adjust control settings as needed. The system's performance has been optimized through servo motor mapping, IMU sensor calibration, and PID controller fine-tuning.

You can view the platform in action through this [demo video](https://drive.google.com/file/d/1M1gVMJAEs7muzSv6spJSX57M_zZE_yGR/view?usp=sharing).

## Conclusion
In conclusion, this project successfully achieved control over the orientation of a platform holder using an Arduino Nano microcontroller. The LabVIEW programming environment provided an intuitive and user-friendly interface for monitoring and controlling the platform. The integration of various components, from servo motor mapping to IMU sensor calibration and PID controller design, resulted in a responsive system.

The servo motors were mapped, taking into consideration the non-upright neutral position and adjusting the PWM range for symmetry. IMU sensor calibration was crucial for obtaining smooth and accurate signals for roll and pitch, with a complementary filter effectively combining accelerometer and gyroscope data. The PID controller design underwent fine-tuning to ensure optimal performance.

The system integration successfully brought together all the components, creating a cohesive and functional unit. The front control panel in LabVIEW provided users with the ability to input desired parameters, view real-time position feedback, and adjust control parameters, enhancing the overall usability of the system.

## Future Work

To suggest further enhancements to improve the system performance, the following future work is recommended:

1. **Develop a mathematical model:** Developing a mathematical model would provide a deeper understanding of the system's behavior and dynamics. This model can be used for analysis, simulation, and advanced control algorithm design.

2. **Implement state feedback:** Implementing state feedback for the position and velocity would improve the system's stability, response times, and disturbance rejection. By measuring or estimating the system's states, the control system can adapt and respond more effectively to different operating conditions and disturbances.

3. **Implement a Kalman filter:** Incorporating a Kalman filter would enhance the estimation of the true state of the system by effectively combining the noisy sensor readings from the accelerometer and gyroscope. This filter would mitigate the limitations of each sensor and provide a more reliable estimate of the roll and pitch angles, leading to improved accuracy and robustness.

By addressing these future work suggestions, the system can be further enhanced and optimized, paving the way for advanced applications and improved performance.

For more detailed information, including circuit wiring, hardware setup, LabVIEW block diagrams, and code implementation, refer to the project documentation.
