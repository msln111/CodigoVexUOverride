#include "robot.h"

#include <cmath>
#include <cstdint>

void initialize() {
    pros::lcd::initialize();

    pros::lcd::set_text(0, "Initializing");

    // Reset the tracking encoders before starting odometry.
    encoderhorizontal.reset_position();
    encodervertical.reset_position();

    // Calibrate the IMU before starting any background task.
    imu.reset();

    while (imu.is_calibrating()) {
        pros::delay(20);
    }

    // Start the coordinate system at the robot's current location.
    reset_odometry(0.0, 0.0, 0.0);

    pros::lcd::set_text(0, "IMU Ready");

    // Start background tasks after the sensors are ready.
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

    // The heading the robot should maintain while driving straight.
    double target_heading = imu.get_heading();

    while (true) {
        int forward =
            master.get_analog(
                pros::E_CONTROLLER_ANALOG_LEFT_Y
            );

        int turn =
            -master.get_analog(
                pros::E_CONTROLLER_ANALOG_RIGHT_X
            );
// The turn axis is inverted so that positive values lorezo es puto
        bool turn_requested =
            std::abs(turn) >= DEADBAND;
        bool drive_requested =
            std::abs(forward) >= DEADBAND;
        if (std::abs(forward) < DEADBAND) {
            forward = 0;
        }

        if (std::abs(turn) < DEADBAND) {
            turn = 0;
        }

        /*
         * When the driver is turning, do not fight the driver.
         * Continuously update the target heading so that when the
         * driver releases the turn stick, the current heading is held.
         */
        if (turn_requested) {
            target_heading = imu.get_heading();
        }

        /*
         * If the robot is not driving, keep updating the target heading.
         * This prevents a correction from being applied after sitting still.
         */
        if (!drive_requested && !turn_requested) {
            target_heading = imu.get_heading();
        }

        double forward_rpm =
            static_cast<double>(forward) *
            (MAX_MOTOR_RPM / 127.0);

        double turn_rpm =
            static_cast<double>(turn) *
            (MAX_TURN_RPM / 127.0);

        double heading_correction = 0.0;

        /*
         * Hold the heading only while driving straight.
         * Do not apply this correction during intentional turning.
         */
        if (drive_requested && !turn_requested) {
            double current_heading = imu.get_heading();

            double heading_error =
                normalize_angle_degrees(
                    target_heading - current_heading
                );

            heading_correction =
                HEADING_CORRECTION_SIGN *
                HEADING_HOLD_KP *
                heading_error;

            heading_correction = clamp_speed(
                heading_correction,
                -MAX_TURN_RPM,
                MAX_TURN_RPM
            );
        }

        /*
         * Positive turn speed makes the right side faster and
         * the left side slower.
         */
        double left_speed =
            forward_rpm -
            turn_rpm -
            heading_correction;

        double right_speed =
            forward_rpm +
            turn_rpm +
            heading_correction;

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