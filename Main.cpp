#include <iostream>
#include <chrono>

class PIDController {
private:
    double kp;      // Proportional gain
    double ki;      // Integral gain
    double kd;      // Derivative gain

    double prev_error;
    double integral;
    std::chrono::steady_clock::time_point prev_time;

public:
    PIDController(double p, double i, double d)
        : kp(p), ki(i), kd(d), prev_error(0.0), integral(0.0) {
        prev_time = std::chrono::steady_clock::now();
    }

    // Compute control output based on setpoint and current value
    double compute(double setpoint, double measured_value) {
        // Calculate time elapsed since last update
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - prev_time;
        double dt = elapsed.count();

        if (dt <= 0.0) dt = 1e-3; // Avoid division by zero

        // Calculate error terms
        double error = setpoint - measured_value;

        // Integral term calculation with simple accumulation
        integral += error * dt;

        // Derivative term calculation
        double derivative = (error - prev_error) / dt;

        // PID output
        double output = kp * error + ki * integral + kd * derivative;

        // Save for next iteration
        prev_error = error;
        prev_time = now;

        return output;
    }

    void reset() {
        prev_error = 0.0;
        integral = 0.0;
        prev_time = std::chrono::steady_clock::now();
    }
};

// Example usage with simulated motor speed
int main() {
    PIDController pid(1.0, 0.1, 0.05);

    double target_speed = 10.0;      // desired speed (units/s)
    double current_speed = 0.0;      // initial speed
    double motor_input = 0.0;        // control variable (e.g., voltage)

    // Simple simulation loop (e.g., 100 iterations)
    for (int i = 0; i < 100; ++i) {
        motor_input = pid.compute(target_speed, current_speed);

        // Simulate motor response: speed changes proportional to input
        // plus some simple damping
        current_speed += motor_input * 0.1;
        current_speed *= 0.9;

        std::cout << "Step " << i << ": Motor input = " << motor_input
                  << ", Speed = " << current_speed << std::endl;

        // Simulate wait e.g., 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
