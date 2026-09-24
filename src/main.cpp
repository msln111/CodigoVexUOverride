#include "robot.h"
#include <cmath>
#include <cstdint>

void initialize() {
    pros::lcd::initialize();

    pros::lcd::set_text(0, "Initializing");

    // Calibrate the IMU before starting odometry.
    imu.reset();

    while (imu.is_calibrating()) {
        pros::delay(20);
    }

    reset_odometry(0.0, 0.0, 0.0);

    pros::lcd::set_text(0, "IMU Ready");

    // Start background tasks after sensor initialization.
    pros::Task odometry_task(
        tareaOdometria,
        nullptr,
        "Odometry Task"
    );

    pros::Task display_task(
        tareaPantalla,
        nullptr,
        "Display Task"
    );
}

void disabled() {
    stop_drive();
}

void competition_initialize() {
    stop_drive();
}

void opcontrol() {
    constexpr int DEADBAND = 10;

    while (true) {
        int forward =
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        int turn =
            -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (std::abs(forward) < DEADBAND) {
            forward = 0;
        }

        if (std::abs(turn) < DEADBAND) {
            turn = 0;
        }

        // Convert controller values from -127..127 to motor RPM.
        double forward_rpm =
            static_cast<double>(forward) *
            (MAX_MOTOR_RPM / 127.0);

        double turn_rpm =
            static_cast<double>(turn) *
            (MAX_TURN_RPM / 127.0);

        double left_speed = forward_rpm - turn_rpm;
        double right_speed = forward_rpm + turn_rpm;

        left_speed = clamp_speed(
            left_speed,
            -MAX_MOTOR_RPM,
            MAX_MOTOR_RPM
        );

        right_speed = clamp_speed(
            right_speed,
            -MAX_MOTOR_RPM,
            MAX_MOTOR_RPM
        );

        set_tank_speed(left_speed, right_speed);

        pros::delay(20);
    }
}