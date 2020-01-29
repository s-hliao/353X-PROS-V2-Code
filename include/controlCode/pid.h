#include "auto.h"
#include "okapi/api.hpp"
using namespace okapi;

bool pidOn  = false;
bool init = true;
bool integOn = true;

QAngle targAngle;

void turnPID(QAngle angle){
  pidOn = true;
  targAngle = angle;
  double error = (odomD->getState().theta-angle).getValue();
  std::uint32_t timer = pros::millis();
  std::uint32_t now = pros::millis();
  init = true;
  while(pros::millis()-timer>150){
    if(error<1||error>1){
      std::uint32_t timer = pros::millis();
    }
    error = (odomD->getState().theta-angle).getValue();
    pros::Task::delay_until(&now, 10);
  }
  pidOn = false;
  fwdD->stop();

}


void turnPID(void*Params){
  double error, integ=0, deriv, lastError;

  std::uint32_t now = pros::millis();
  while(true){
    if(pidOn){
      error = (targAngle-odomD->getState().theta).getValue();
      if(init){
        lastError=error;
        init = false;
      }
      if(integOn) integ+=error;
      deriv = error-lastError;

      double pow = tP*error+tI*integ+tD*deriv;

      if(1<pow){
        fwdD->tank(1, -1);
      }  else if(-1>pow){
        fwdD->tank(-1, 1);
      } else{
        fwdD->tank(pow, -pow);
      }


      lastError = error;
    } else{
      integ = 0;
    }
    pros::Task::delay_until(&now, 10);
  }
}

void makePath(std::shared_ptr<AsyncMotionProfileController> profile,
              std::initializer_list<PathfinderPoint> waypoints,
              std::string ID){
   profile->generatePath(waypoints, ID);

}
