#include "controlCode/control.h"
#include "pros/apix.h"
#include "pros/rtos.hpp"

using namespace okapi;
void initialize() {
	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");

}

void disabled() {}
void competition_initialize() {

}

void opcontrol() {
	pros::lcd::initialize();

	slowProfile->flipDisable(true);
	motorInit();
	pros::Task driveT(tankDriveControl, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Drive Control");
  pros::Task liftT(liftControl, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Lift Control");
  pros::Task rollerT(rollerControl, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Roller Control");
  pros::Task trayT(trayControl, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Tray Control");
  pros::Task trayPowT(trayPow, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Tray Pow");
  pros::Task filterT(filterPot, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Filter Potentiometer");

}
