#include "controlCode/pid.h"
#include "pros/motors.h"

using namespace okapi;

int trayOut = 1150;
int trayIn = 3000;
int trayMid = 2200;

int liftDown = 90;
int liftLow =2000;
int liftHigh = 3700;

bool macro = false;


void motorInit(){
  LRoller.set_brake_mode(MOTOR_BRAKE_BRAKE);
  RRoller.set_brake_mode(MOTOR_BRAKE_BRAKE);
  Tray.set_brake_mode(MOTOR_BRAKE_BRAKE);
  Lift.set_brake_mode(MOTOR_BRAKE_HOLD);
  fwdD->setBrakeMode(AbstractMotor::brakeMode::brake);
}


void tankDriveControl(void*param){
  std::uint32_t now = pros::millis();
  while(true){ //tank control Left and right analog sticks
      if(control.get_digital(DIGITAL_X)){
        while(((double)control.get_analog(ANALOG_LEFT_Y))/127<.03 && ((double)control.get_analog(ANALOG_LEFT_Y))/127>-.03 &&
        ((double)control.get_analog(ANALOG_RIGHT_Y))/127<.03 && ((double)control.get_analog(ANALOG_RIGHT_Y))/127>-.03){
          fwdD->tank(1,-1);
        }
      }
    	fwdD->tank(((double)control.get_analog(ANALOG_LEFT_Y))/127,
        ((double)control.get_analog(ANALOG_RIGHT_Y))/127, .03);

      pros::Task::delay_until(&now, 10);

  }
}

void cubeOut(){
  if(liftRead<liftDown+100){
    LRoller.set_zero_position(0);
    RRoller.set_zero_position(0);
    LRoller.move_absolute(-500, -200);
    RRoller.move_absolute(-500, -200);
  }
  macro=true;
}


void liftControl(void* param){
  std::uint32_t now =  pros::millis();
  while(true){

    if(control.get_digital(DIGITAL_L1)){ //Lift up
      Lift.move_velocity(200);
      Lift.set_brake_mode(MOTOR_BRAKE_HOLD);
    } else if(control.get_digital(DIGITAL_L2)){ //lift down
      Lift.move_velocity(-200);
      Lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    } else if(control.get_digital(DIGITAL_UP)){
      Lift.move_velocity(-50);
    }else if(control.get_digital(DIGITAL_Y)){ //short lift macro
      Lift.set_brake_mode(MOTOR_BRAKE_HOLD);
      cubeOut();
      if(liftRead<liftLow){
        while(liftRead<liftLow){
          if(control.get_digital(DIGITAL_L1)||control.get_digital(DIGITAL_L2)||control.get_digital(DIGITAL_RIGHT)||control.get_digital(DIGITAL_B)){
            break;
          }
          Lift.move_velocity(200);
          pros::Task::delay_until(&now, 10);
        }
      } else if(liftRead>liftLow){
        while(liftRead>liftLow){
          if(control.get_digital(DIGITAL_L1)||control.get_digital(DIGITAL_L2)||control.get_digital(DIGITAL_Y)||control.get_digital(DIGITAL_B)){
            break;
          }
          Lift.move_velocity(-200);
          pros::Task::delay_until(&now, 10);
        }
      }
      macro =false;
      Lift.move_velocity(0);
      while(control.get_digital(DIGITAL_Y)){
        pros::Task::delay_until(&now, 10);
      };
    } else if(control.get_digital(DIGITAL_B)){//high lift macro
      Lift.set_brake_mode(MOTOR_BRAKE_HOLD);
      cubeOut();
      if(liftRead<liftHigh){
        while(liftRead<liftHigh){
          if(control.get_digital(DIGITAL_L1)||control.get_digital(DIGITAL_L2)||control.get_digital(DIGITAL_Y)||control.get_digital(DIGITAL_RIGHT)){
            break;
          }
          Lift.move_velocity(200);
          pros::Task::delay_until(&now, 10);
        }
      } else if(liftRead>liftHigh){
        while(liftRead>liftHigh){
          if(control.get_digital(DIGITAL_L1)||control.get_digital(DIGITAL_L2)||control.get_digital(DIGITAL_Y)||control.get_digital(DIGITAL_RIGHT)){
            break;
          }
          Lift.move_velocity(-200);
          pros::Task::delay_until(&now, 10);
        }
      }
      macro =false;
      Lift.move_velocity(0);
      while(control.get_digital(DIGITAL_B)){
        pros::Task::delay_until(&now, 10);
      };
    } else if(control.get_digital(DIGITAL_RIGHT)){ //lift down
      Lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
      if(liftRead<liftDown){
        while(liftRead<liftDown){
          if(control.get_digital(DIGITAL_L1)||control.get_digital(DIGITAL_L2)||control.get_digital(DIGITAL_Y)||control.get_digital(DIGITAL_B)){
            break;
          }
          Lift.move_velocity(200);
          pros::Task::delay_until(&now, 10);
        }
      } else if(liftRead>liftDown){
        while(liftRead>liftDown){
          if(control.get_digital(DIGITAL_L1)||control.get_digital(DIGITAL_L2)||control.get_digital(DIGITAL_Y)||control.get_digital(DIGITAL_B)){
            break;
          }
          Lift.move_velocity(-200);
          pros::Task::delay_until(&now, 10);
        }
      }
      macro =false;
      Lift.move_velocity(0);
      while(control.get_digital(DIGITAL_RIGHT)){
        pros::Task::delay_until(&now, 10);
      };
    }else if (control.get_digital(DIGITAL_UP)){ // tray moving out
      Lift.set_brake_mode(MOTOR_BRAKE_HOLD);
    } else if(control.get_digital(DIGITAL_R1)){
      Lift.set_brake_mode(MOTOR_BRAKE_BRAKE);

    }
    else{
      Lift.move_velocity(0);
    }
    pros::Task::delay_until(&now, 10);
  }
}

void rollerControl(void* param){
  std::uint32_t now = pros::millis();
  while(true){

    if(control.get_digital(DIGITAL_R1)){
      LRoller.move_velocity(200);
      RRoller.move_velocity(200);

    } else if(control.get_digital(DIGITAL_R2)){
      LRoller.move_velocity(-200);
      RRoller.move_velocity(-200);
      Lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    } else if(control.get_digital(DIGITAL_A)){
      LRoller.move_velocity(-125);
      RRoller.move_velocity(-125);
    } else if(!macro){
      LRoller.move_velocity(0);
      RRoller.move_velocity(0);

    }
    pros::Task::delay_until(&now, 10);
  }
}

int outtake= 50;
bool trayWorking = true;

void trayControl(void* param){
  std::uint32_t now = pros::millis();
  while(true){
     if(control.get_digital(DIGITAL_UP)){
      Tray.move_velocity(outtake);
      Tray.set_brake_mode(MOTOR_BRAKE_BRAKE);
      Lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    } else if(control.get_digital(DIGITAL_LEFT)){
      Tray.move_velocity(-100);
      Tray.set_brake_mode(MOTOR_BRAKE_BRAKE);
      Lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
    } else if(control.get_digital(DIGITAL_DOWN)){
       while(trayRead<trayIn){
         Tray.move_velocity(-100);
         if(control.get_digital(DIGITAL_UP)||control.get_digital(DIGITAL_RIGHT)){
           break;
         }
       }
       while(control.get_digital(DIGITAL_DOWN)){
         pros::Task::delay_until(&now, 10);
       }
    }
    else{
      Tray.move_velocity(0);
    }
    pros::Task::delay_until(&now, 10);
  }
}

void trayPow(void* param){
  std::uint32_t now = pros::millis();
  while(true){
    if(trayRead>trayMid){
      outtake = 100;
    }
    else if (trayRead>trayOut){
      outtake = 25+30*(trayRead-trayOut)/((trayMid)-trayOut);
    }
    else{
      outtake = 0;
    }

    pros::Task::delay_until(&now, 10);
  }

}



void filterPot(void*param){
  std::uint32_t now = pros::millis();
  while (true) {


    liftRead = liftPotF.filter(liftPot.get_value());
    trayRead = trayPotF.filter(trayPot.get_value());

    pros::lcd::print(1, "X Pos: %f", odomD->getState().x.convert(foot));
		pros::lcd::print(2, "Y Pos: %f", odomD->getState().y.convert(foot));
		pros::lcd::print(3, "Angle: %f", odomD->getState().theta.convert(degree));
    pros::lcd::print(4, "Tray Filter: %f", trayRead);
    pros::lcd::print(5, "Lift Filter: %f", liftRead);
    pros::lcd::print(6, "Tray: %d", trayPot.get_value());
    pros::lcd::print(7, "Lift: %d", liftPot.get_value());


    pros::Task::delay_until(&now, 10);
  }
}
