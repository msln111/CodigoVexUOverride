#include "robot.h"

#include <cmath>
#include <cstdint>
#include <string>

// --------------------------------------------------
// Helper: Read odometry safely
// --------------------------------------------------

void read_odometry(double &x, double &y, double &theta) {
    get_position(x, y, theta);
}

// --------------------------------------------------
// Helper: Print debug info to LCD
// --------------------------------------------------

void print_debug(
    int line,
    const std::string &label,
    double value
) {
    pros::lcd::set_text(
        line,
        label + ": " + std::to_string(value)
    );
}

// --------------------------------------------------
// Turn to a heading
// --------------------------------------------------

void girarAngulo(double objetivoTheta) {
    constexpr double ANGLE_TOLERANCE = 2.0;
    constexpr double MAX_TURN_SPEED = 250.0;
    constexpr double MIN_TURN_SPEED = 35.0;
    constexpr uint32_t TIMEOUT_MS = 4000;
    constexpr double TURN_KP = 4.0;

    uint32_t start_time = pros::millis();

    while (
        pros::millis() - start_time <
        TIMEOUT_MS
    ) {
        double current_x;
        double current_y;
        double current_theta;

        read_odometry(
            current_x,
            current_y,
            current_theta
        );

        double error =
            normalize_angle_degrees(
                objetivoTheta -
                current_theta
            );

        // Print debug info every iteration
        print_debug(5, "Target Ang", objetivoTheta);
        print_debug(6, "Current Ang", current_theta);
        print_debug(7, "Error", error);

        if (
            std::fabs(error) <=
            ANGLE_TOLERANCE
        ) {
            break;
        }

        double turn_speed =
            TURN_KP *
            error;

        turn_speed =
            clamp_speed(
                turn_speed,
                -MAX_TURN_SPEED,
                MAX_TURN_SPEED
            );

        if (
            std::fabs(turn_speed) <
            MIN_TURN_SPEED
        ) {
            turn_speed =
                turn_speed >= 0.0
                    ? MIN_TURN_SPEED
                    : -MIN_TURN_SPEED;
        }

        set_tank_speed(
            -turn_speed,
            turn_speed
        );

        pros::delay(20);
    }

    stop_drive();
}

// --------------------------------------------------
// Move to a field coordinate
// --------------------------------------------------

void moverAPunto(
    double objetivoX,
    double objetivoY,
    double objetivoTheta
) {
    constexpr double POSITION_TOLERANCE = 3.0;
    constexpr double MAX_DRIVE_SPEED = 400.0;
    constexpr double MAX_TURN_SPEED = 220.0;
    constexpr uint32_t TIMEOUT_MS = 5000;
    constexpr double DISTANCE_KP = 2.0;
    constexpr double ANGLE_KP = 4.0;
    constexpr double MIN_DRIVE_SPEED = 40.0;

    uint32_t start_time = pros::millis();

    while (
        pros::millis() - start_time <
        TIMEOUT_MS
    ) {
        double actualX;
        double actualY;
        double actualTheta;

        read_odometry(
            actualX,
            actualY,
            actualTheta
        );

        double errorX =
            objetivoX -
            actualX;

        double errorY =
            objetivoY -
            actualY;

        double distance_error =
            std::sqrt(
                (errorX * errorX) +
                (errorY * errorY)
            );

        // Print debug info
        print_debug(3, "Target X", objetivoX);
        print_debug(4, "Target Y", objetivoY);
        print_debug(5, "Actual X", actualX);
        print_debug(6, "Actual Y", actualY);
        print_debug(7, "Dist Err", distance_error);

        if (
            distance_error <=
            POSITION_TOLERANCE
        ) {
            break;
        }

        double target_angle =
            std::atan2(
                errorY,
                errorX
            ) *
            180.0 /
            M_PI;

        double heading_error =
            normalize_angle_degrees(
                target_angle -
                actualTheta
            );

        double heading_scale =
            std::cos(
                heading_error *
                M_PI /
                180.0
            );

        if (heading_scale < 0.0) {
            heading_scale = 0.0;
        }

        double forward_speed =
            DISTANCE_KP *
            distance_error *
            heading_scale;

        if (
            std::fabs(forward_speed) <
            MIN_DRIVE_SPEED &&
            forward_speed != 0.0
        ) {
            forward_speed =
                forward_speed >= 0.0
                    ? MIN_DRIVE_SPEED
                    : -MIN_DRIVE_SPEED;
        }

        double turn_speed =
            ANGLE_KP *
            heading_error;

        forward_speed =
            clamp_speed(
                forward_speed,
                -MAX_DRIVE_SPEED,
                MAX_DRIVE_SPEED
            );

        turn_speed =
            clamp_speed(
                turn_speed,
                -MAX_TURN_SPEED,
                MAX_TURN_SPEED
            );

        double left_speed =
            forward_speed -
            turn_speed;

        double right_speed =
            forward_speed +
            turn_speed;

        set_tank_speed(
            left_speed,
            right_speed
        );

        pros::delay(20);
    }

    stop_drive();

    girarAngulo(
        objetivoTheta
    );
}

// --------------------------------------------------
// Autonomous routine
// --------------------------------------------------

void autonomous() {
    // Start position: (0, 0) with heading = 90°
    // This means robot is facing FORWARD (positive Y)
    reset_odometry(
        0.0,
        0.0,
        90.0
    );

    pros::delay(200); // Let odometry stabilize

    // Move 24 inches forward in the positive Y direction
    // and end facing 0° (rotated to face right/positive X)
    moverAPunto(
        0.0,    // Target X (no lateral movement)
        24.0,   // Target Y (24 inches forward)
        90.0     // Target heading (face right)
    );

    pros::delay(500); // Pause to inspect position

    stop_drive();
}
