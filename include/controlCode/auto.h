#include "robotConfig.h"
#include "okapi/api.hpp"
using namespace okapi;

double kP = .0009, kI = .000000, kD = .0000001;
double tP = .001, tI = .000000, tD = .0000001;
std::shared_ptr<Logger> lg;

auto odomD = ChassisControllerBuilder()
  .withMotors(MotorGroup({LFDrive, LBDrive}),MotorGroup({RFDrive, RBDrive}))
  .withDimensions(AbstractMotor::gearset::blue, ChassisScales({1.794642857_in, 11.375_in}, imev5BlueTPR)) //scaled 4.1875
  .withLogger(std::make_shared<Logger>(
            TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
            "/ser/sout", // Output to the PROS terminal
            Logger::LogLevel::debug // Most verbose log level
  ))
  .withGains({kP,kI,kD}, {tP,tI,tD})

  .withOdometry()
  .buildOdometry();

std::shared_ptr<ChassisModel> fwdD = odomD->getModel();


auto slowProfile = AsyncMotionProfileControllerBuilder()
  .withOutput(odomD)
  .withLimits({.7, 2, 10})
  .buildMotionProfileController();


PathfinderPoint getCurPoint(){
    return PathfinderPoint{odomD->getState().x, odomD->getState().y, odomD->getState().theta};
}

PathfinderPoint getCurPointBackwards(){
    return PathfinderPoint{-(odomD->getState().x), odomD->getState().y, odomD->getState().theta};
}
PathfinderLimits slow = {.55, 2, 10};
PathfinderLimits fast = {6, 12, 20};


void driveTime(double speed, int time){
  std::uint32_t timer = pros::millis();
  std::uint32_t now = pros::millis();

  fwdD->tank(speed/600, speed/600);
  while(pros::millis()-timer<time){
    pros::Task::delay_until(&now, 2);
  }
}


void driveSpeed(double speed, int pos){
  std::uint32_t now = pros::millis();
  fwdD->resetSensors();

  fwdD->tank(speed/600, speed/600);
  if(pos>0){
    while((fwdD->getSensorVals()[0]+fwdD->getSensorVals()[1])/2<pos){
      pros::Task::delay_until(&now, 2);
    }
  } else if(pos<0){
    while((fwdD->getSensorVals()[0]+fwdD->getSensorVals()[1])/2>pos){
      pros::Task::delay_until(&now, 2);
    }
  }
}

void liftTo(int x){
  if(liftPotF.getOutput()<x){
    while(liftPotF.getOutput()<x){
      Lift.move_velocity(100);
    }
  } else if(liftPotF.getOutput()>x){
    while(liftPotF.getOutput()>x){
      Lift.move_velocity(-100);
    }
  }

}
