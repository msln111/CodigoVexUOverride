#include "main.h"
#include "EZ-Template/api.hpp"
#include "robot.h"
#include <string>
#include <cmath>

double globalHorizontal=0.0;
double globalVertical=0.0;
double globalTheta=0.0;

pros::Mutex odometry_mutex;

pros::Rotation encoderhorizontal(11);
pros::Rotation encodervertical(12);
pros::IMU imu(10);

static const double diametroRueda = 3.25; // Diámetro de la rueda de tracking en pulgadas
static const double tickPorRevolucion = 36000.0; // Número de ticks por revolución de la rueda de tracking
static const double circunferenciaRueda = M_PI * diametroRueda; // Circunferencia de la rueda de tracking en pulgadas
static const double pulgadasPorTick = circunferenciaRueda / tickPorRevolucion; // Pulgadas por tick de la rueda de tracking
static const double LX=1;//Distancia del encoder al centro de rotacion
static const double LY=1.25;//Distancia del encoder al centro de rotacion

static double ultimaPosicionHorizontal=0.0, ultimaPosicionVertical=0.0, ultimaPosicionTheta=0.0;

pros::MotorGroup leftMotors({-1, 2, -3, -13}); // No invertir los motores izquierdos
pros::MotorGroup rightMotors({5, -6, 7, 8}); // Invertir los motores derechos

pros::Controller master(pros::E_CONTROLLER_MASTER);

void get_position(double &x, double &y, double &theta) {
    odometry_mutex.take();
    x = globalHorizontal;
    y = globalVertical;
    theta = globalTheta;
    odometry_mutex.give();
}

void set_tank_speed(double left_speed, double right_speed) {
    leftMotors.move_velocity(left_speed);
    rightMotors.move_velocity(right_speed);
}

void tareaOdometria(void* param) {
       encoderhorizontal.reset_position();
       encodervertical.reset_position();
       imu.reset();
       while (imu.is_calibrating()) {
           pros::delay(20);
       }
    ultimaPosicionHorizontal = encoderhorizontal.get_position();
    ultimaPosicionVertical = encodervertical.get_position();
    ultimaPosicionTheta = imu.get_heading() * M_PI / 180.0; // Convertir a radianes

    while (true) {
        double encHorizontal = encoderhorizontal.get_position();
        double encVertical = encodervertical.get_position();
        double imuTheta = imu.get_heading() * M_PI / 180.0; // Convertir a radianes

        double deltaHorizontal = (encHorizontal - ultimaPosicionHorizontal)*pulgadasPorTick;
        double deltaVertical = (encVertical - ultimaPosicionVertical)*pulgadasPorTick;
        double deltaTheta = imuTheta - ultimaPosicionTheta;

        double correccionHorizontal = deltaHorizontal - (LY * deltaTheta);
        double correccionVertical = deltaVertical + (LX * deltaTheta);
        double anguloMedio = ultimaPosicionTheta + (deltaTheta / 2.0);

        double cos_t = cos(anguloMedio);
        double sin_t = sin(anguloMedio);
        double deltaHorizontalCampo = (correccionHorizontal * cos_t) - (correccionVertical * sin_t);
        double deltaVerticalCampo = (correccionHorizontal * sin_t) + (correccionVertical * cos_t);

        odometry_mutex.take();
        globalHorizontal += deltaHorizontalCampo;
        globalVertical += deltaVerticalCampo;
        globalTheta += deltaTheta;
        odometry_mutex.give();

        ultimaPosicionHorizontal = encHorizontal;
        ultimaPosicionVertical = encVertical;
        ultimaPosicionTheta = imuTheta;

        pros::delay(10);
    }
}

void tareaPantalla(void* param) {
    while (true) {
        double x, y, theta;
        get_position(x, y, theta);
        double thetaGrados = theta * (180.0 / M_PI); // Convertir a grados

        pros::lcd::clear_line(0);
        pros::lcd::clear_line(1);
        pros::lcd::clear_line(2);

        pros::lcd::set_text(0, "X :" + std::to_string(x) + " in");
        pros::lcd::set_text(1, "Y: " + std::to_string(y) + " in");
        pros::lcd::set_text(2, "Theta: " + std::to_string(thetaGrados) + " deg");

        pros::delay(50);
    }
}