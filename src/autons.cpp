#include "robot.h"

#include <cmath>
#include <cstdint>

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

        get_position(
            current_x,
            current_y,
            current_theta
        );

        double error =
            normalize_angle_degrees(
                objetivoTheta -
                current_theta
            );

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
    // Tune these if the robot overshoots or undershoots
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

        get_position(
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

        // Prevent the robot from moving
        // so slowly it stalls.
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
  
    reset_odometry(0, 0 ,90.0);   // Move 24 inches in the positive Y direction.
    moverAPunto(0.0, 5.0, 0.0);


    stop_drive();
}
