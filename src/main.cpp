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

// ========================
// Joystick sensitivity curve
// ========================
//
// This function applies a smooth curve to joystick input.
// - Small stick movements → small motor speed (fine control)
// - Large stick movements → proportional speed
// - Full stick → maximum speed
//
// Experiment with SENSITIVITY_POWER to tune responsiveness:
//   1.0 = linear (no curve)
//   1.5 = slight curve (small movements are easier to control)
//   2.0 = quadratic (small movements = very fine control)
//   3.0 = cubic (aggressive curve, requires full stick for max speed)

double apply_joystick_curve(
    double stick_value,
    double sensitivity_power
) {
    // Normalize to -1.0 to 1.0
    double normalized = stick_value / 127.0;

    // Preserve sign, apply power to magnitude
    double sign = (normalized >= 0.0) ? 1.0 : -1.0;
    double magnitude =
        std::pow(
            std::abs(normalized),
            sensitivity_power
        );

    // Scale back to full range
    return sign * magnitude * 127.0;
}

void opcontrol() {
    constexpr int DEADBAND = 10;

    /*
     * JOYSTICK SENSITIVITY TUNING
     *
     * Adjust these values to match your driving style:
     *   1.5 = gentle curve (easier control at low speeds)
     *   2.0 = moderate curve (good for most robots)
     *   2.5 = aggressive curve (requires more stick deflection for full speed)
     */
    constexpr double FORWARD_SENSITIVITY = 2.0;
    constexpr double TURN_SENSITIVITY = 2.0;

    /*
     * TURN SMOOTHNESS SETTINGS
     *
     * These control how quickly the robot rotates.
     * Lower values = smoother but slower turns
     * Higher values = faster but potentially jerkier turns
     */
    constexpr double MAX_TURN_ACCEL = 50.0;  // RPM per update (20ms)

    double previous_turn_speed = 0.0;

    while (true) {
        // Read raw joystick values
        int forward_raw =
            master.get_analog(
                pros::E_CONTROLLER_ANALOG_LEFT_Y
            );

        int turn_raw =
            -master.get_analog(
                pros::E_CONTROLLER_ANALOG_RIGHT_X
            );

        // Apply deadband
        if (std::abs(forward_raw) < DEADBAND) {
            forward_raw = 0;
        }

        if (std::abs(turn_raw) < DEADBAND) {
            turn_raw = 0;
        }

        // Apply joystick curve for sensitivity
        double forward_curved =
            apply_joystick_curve(
                forward_raw,
                FORWARD_SENSITIVITY
            );

        double turn_curved =
            apply_joystick_curve(
                turn_raw,
                TURN_SENSITIVITY
            );

        // Convert to RPM
        double forward_rpm =
            forward_curved *
            (MAX_MOTOR_RPM / 127.0);

        double turn_rpm =
            turn_curved *
            (MAX_TURN_RPM / 127.0);

        /*
         * TURN SMOOTHING (acceleration limiting)
         *
         * Gradually change the turn speed instead of jumping instantly.
         * This makes turns much smoother and more controllable.
         */
        double turn_delta =
            turn_rpm - previous_turn_speed;

        if (turn_delta > MAX_TURN_ACCEL) {
            turn_rpm =
                previous_turn_speed +
                MAX_TURN_ACCEL;
        } else if (turn_delta < -MAX_TURN_ACCEL) {
            turn_rpm =
                previous_turn_speed -
                MAX_TURN_ACCEL;
        }

        previous_turn_speed = turn_rpm;

        /*
         * Tank drive calculation
         *
         * Left motor gets forward minus turn
         * Right motor gets forward plus turn
         */
        double left_speed =
            forward_rpm - turn_rpm;

        double right_speed =
            forward_rpm + turn_rpm;

        // Clamp to motor limits
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