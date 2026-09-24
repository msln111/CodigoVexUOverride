#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <cmath>
#include <string>
#include "EZ-Template/api.hpp"

class customPID {
    public:
    double kp, ki, kd;
    double integral, previous_error;
    double max_integral;

        customPID(double p, double i, double d, double max_i = 1000)
        : kp(p), ki(i), kd(d), integral(0.0), previous_error(0.0), max_integral(max_i) {}

        double calculate(double error, double dt){
            integral += error * dt;
            
            if (integral > max_integral) integral = max_integral;
            if (integral < -max_integral) integral = -max_integral;
            double derivative = (error - previous_error) / dt;
            previous_error = error;
            return kp * error + ki * integral + kd * derivative;
        }
    void reset() {
        integral = 0.0;
        previous_error = 0.0;
    }
};

extern double globalHorizontal, globalVertical, globalTheta;
extern pros::Mutex odometry_mutex;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Controller master;
extern pros::IMU imu;
extern pros::Rotation encoderhorizontal;
extern pros::Rotation encodervertical;

void get_position(double &x, double &y, double &theta);
void set_tank_speed(double left_speed, double right_speed);
void tareaOdometria(void* param);
void tareaPantalla(void* param);

#endif // ROBOT_H