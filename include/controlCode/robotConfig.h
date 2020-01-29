#include "main.h"
#include "pros/rtos.h"
#include "pros/motors.h"
#include "okapi/api.hpp"


using namespace okapi;

const int8_t LFDPort  = 9;
const int8_t LBDPort = 10;
const int8_t RFDPort = 1;
const int8_t RBDPort = 2;

const Motor LFDrive(LFDPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
const Motor LBDrive(LBDPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
const Motor RFDrive(RFDPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
const Motor RBDrive(RBDPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
pros::Motor LRoller(19, MOTOR_GEARSET_18, false);
pros::Motor RRoller(12, MOTOR_GEARSET_18, true); //must run opposite of Left ROller
pros::Motor Lift(20, MOTOR_GEARSET_18, false);
pros::Motor Tray(3, MOTOR_GEARSET_36, true);

pros::Controller control (CONTROLLER_MASTER);

pros::ADIPotentiometer trayPot('b');
pros::ADIPotentiometer liftPot('c');

okapi::MedianFilter<5> trayPotF;
okapi::MedianFilter<5> liftPotF;

double trayRead=0;
double liftRead = 0;
