#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <cmath>
#include <cstdint>

// -----------------------------
// Robot constants
// -----------------------------

constexpr double MAX_MOTOR_RPM = 600.0;
constexpr double MAX_TURN_RPM = 300.0;

// -----------------------------
// Simple PID controller
// -----------------------------

class customPID {
public:
    double kp;
    double ki;
    double kd;
    double integral;
    double previous_error;
    double max_integral;

    customPID(double p, double i, double d, double max_i = 1000.0)
        : kp(p),
          ki(i),
          kd(d),
          integral(0.0),
          previous_error(0.0),
          max_integral(max_i) {}

    double calculate(double error, double dt) {
        if (dt <= 0.0) {
            dt = 0.01;
        }

        integral += error * dt;

        if (integral > max_integral) {
            integral = max_integral;
        }

        if (integral < -max_integral) {
            integral = -max_integral;
        }

        double derivative = (error - previous_error) / dt;
        previous_error = error;

        return (kp * error) + (ki * integral) + (kd * derivative);
    }

    void reset() {
        integral = 0.0;
        previous_error = 0.0;
    }
};

// -----------------------------
// Global robot objects
// -----------------------------

extern double globalHorizontal;
extern double globalVertical;
extern double globalTheta;

extern pros::Mutex odometry_mutex;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Controller master;
extern pros::IMU imu;

extern pros::Rotation encoderhorizontal;
extern pros::Rotation encodervertical;

// -----------------------------
// Helper functions
// -----------------------------

double clamp_speed(double value, double minimum, double maximum);

double normalize_angle_degrees(double angle);

void get_position(double &x, double &y, double &theta);

void reset_odometry(double x = 0.0,
                    double y = 0.0,
                    double theta_degrees = 0.0);

void set_tank_speed(double left_speed, double right_speed);

void stop_drive();

// -----------------------------
// Background tasks
// -----------------------------

void tareaOdometria(void *param);
void tareaPantalla(void *param);

// -----------------------------
// Autonomous functions
// -----------------------------

void girarAngulo(double objetivoTheta);
void moverAPunto(double objetivoX,
                 double objetivoY,
                 double objetivoTheta);

#endif