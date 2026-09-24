#include "robot.h"

#include <cmath>
#include <cstdint>

// -----------------------------
// Turn the robot to a heading
// -----------------------------

void girarAngulo(double objetivoTheta) {
    constexpr double ANGLE_TOLERANCE = 2.0;
    constexpr double MAX_TURN_SPEED = 300.0;
    constexpr uint32_t TIMEOUT_MS = 4000;

    const double kP = 4.0;

    uint32_t start_time = pros::millis();

    while (pros::millis() - start_time < TIMEOUT_MS) {
        double current_x;
        double current_y;
        double current_theta;

        get_position(
            current_x,
            current_y,
            current_theta
        );

        double error =
            normalize_angle_degrees(
                objetivoTheta - current_theta
            );

        if (std::fabs(error) <= ANGLE_TOLERANCE) {
            break;
        }

        double turn_speed = kP * error;

        turn_speed = clamp_speed(
            turn_speed,
            -MAX_TURN_SPEED,
            MAX_TURN_SPEED
        );

        set_tank_speed(-turn_speed, turn_speed);

        pros::delay(20);
    }

    stop_drive();
}

// -----------------------------
// Move to a field coordinate
// -----------------------------

void moverAPunto(
    double objetivoX,
    double objetivoY,
    double objetivoTheta
) {
    constexpr double POSITION_TOLERANCE = 1.0;
    constexpr double ANGLE_TOLERANCE = 3.0;
    constexpr double MAX_DRIVE_SPEED = 450.0;
    constexpr double MAX_TURN_SPEED = 250.0;
    constexpr uint32_t TIMEOUT_MS = 8000;

    const double distance_kP = 12.0;
    const double angle_kP = 4.0;

    uint32_t start_time = pros::millis();

    while (pros::millis() - start_time < TIMEOUT_MS) {
        double actualX;
        double actualY;
        double actualTheta;

        get_position(
            actualX,
            actualY,
            actualTheta
        );

        double errorX = objetivoX - actualX;
        double errorY = objetivoY - actualY;

        double distance_error =
            std::sqrt(
                (errorX * errorX) +
                (errorY * errorY)
            );

        if (distance_error <= POSITION_TOLERANCE) {
            break;
        }

        // Direction from the robot to the target in field coordinates.
        double target_angle =
            std::atan2(errorY, errorX) *
            180.0 / M_PI;

        double heading_error =
            normalize_angle_degrees(
                target_angle - actualTheta
            );

        // Drive more slowly when the robot is not facing the target.
        double heading_scale =
            std::cos(
                heading_error * M_PI / 180.0
            );

        if (heading_scale < 0.0) {
            heading_scale = 0.0;
        }

        double forward_speed =
            distance_kP *
            distance_error *
            heading_scale;

        double turn_speed =
            angle_kP * heading_error;

        forward_speed = clamp_speed(
            forward_speed,
            -MAX_DRIVE_SPEED,
            MAX_DRIVE_SPEED
        );

        turn_speed = clamp_speed(
            turn_speed,
            -MAX_TURN_SPEED,
            MAX_TURN_SPEED
        );

        double left_speed =
            forward_speed - turn_speed;

        double right_speed =
            forward_speed + turn_speed;

        set_tank_speed(left_speed, right_speed);

        pros::delay(20);
    }

    stop_drive();

    // Rotate to the requested final heading.
    girarAngulo(objetivoTheta);
}

// -----------------------------
// Autonomous routine
// -----------------------------

void autonomous() {
    reset_odometry(0.0, 0.0, 0.0);

    moverAPunto(24.0, 0.0, 0.0);
    moverAPunto(24.0, 24.0, 90.0);
    moverAPunto(0.0, 24.0, 180.0);
    moverAPunto(0.0, 0.0, 270.0);

    stop_drive();
}