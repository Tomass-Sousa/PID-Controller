# PID Controller in C++

## Overview

This project implements a basic **PID controller** in C++ to regulate a motor's speed.  
The PID (Proportional-Integral-Derivative) controller is a classic feedback loop algorithm widely used in robotics and control systems for achieving stable and accurate control.

---

## How It Works

### PID Components

- **Proportional (P)**: Reacts proportionally to the current error (difference between desired setpoint and measured value).  
- **Integral (I)**: Accumulates past errors to eliminate steady-state offset.  
- **Derivative (D)**: Predicts future error trend based on the rate of change of the error, providing damping.

### Algorithm Flow

1. Measure the current value of the controlled variable (e.g., motor speed).
2. Calculate the error: `error = setpoint - measured_value`.
3. Compute the integral of the error over time.
4. Calculate the derivative (rate of error change).
5. Combine the three terms with their respective gains (kp, ki, kd) to produce the control output.
6. Apply the control output to the system.
7. Repeat at regular time intervals.

---

## Code Explanation

- **Class `PIDController`**:  
  Encapsulates PID parameters and internal state (previous error, integral sum, time tracking).  
  The method `compute(setpoint, measured_value)` returns the control output based on the current error and elapsed time.

- **Time Handling**:  
  Uses `std::chrono::steady_clock` to calculate precise elapsed time (`dt`) between successive calls for accurate integration and differentiation.

- **Integral Windup Consideration**:  
  This basic implementation does not handle integral windup explicitly (where integral grows excessively). In real systems, clamping or resetting integral terms is recommended.

- **Example in `main()`**:  
  Simulates a motor speed controlled by the PID output. The motor speed responds to the control input with some delay and damping.

---

## How to Compile and Run

Make sure you have a C++11-compatible compiler.

```bash
g++ -std=c++11 -o pid_controller pid_controller.cpp
./pid_controller
```

## Potential Extensions

- Add integral windup protection by limiting the integral term.
- Add output saturation limits to simulate actuator limits.
- Integrate with real sensors and actuators (e.g., via ROS topics).
- Implement tuning tools to optimize PID gains automatically.

## References 

- PID Controller Wikipedia
- Modern Control Systems textbooks
- Robotics control lectures and tutorials
