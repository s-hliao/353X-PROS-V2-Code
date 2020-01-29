#include "main.h"
#include "okapi/api.hpp"


using namespace okapi;
extern std::shared_ptr<DefaultOdomChassisController> odomD;
extern std::shared_ptr<ChassisModel>  fwdD ;
extern std::shared_ptr<AsyncMotionProfileController>  slowProfile ;
extern PathfinderLimits fast;
extern PathfinderLimits slow;

extern void motorInit();
extern void driveSpeed(double speed, int pos);
extern void driveTime(double speed, int time);

extern PathfinderPoint getCurPoint();
extern PathfinderPoint getCurPointBackwards();
extern void filterPot(void*param);

extern pros::ADIPotentiometer trayPot;
extern pros::ADIPotentiometer liftPot;

extern okapi::MedianFilter<5> trayPotF;
extern okapi::MedianFilter<5> liftPotF;

extern double trayRead;
extern double liftRead;

extern pros::Motor LRoller;
extern pros::Motor RRoller;
extern pros::Motor Tray;
extern pros::Motor Lift;

bool isRed = true;

extern int trayIn;
extern int trayMid;
extern int trayOut;


void printOdom(void*params){
  std::uint32_t now = pros::millis();
  while(true){
    pros::lcd::print(1, "X Pos: %f", odomD->getState().x.convert(foot));
    pros::lcd::print(2, "Y Pos: %f", odomD->getState().y.convert(foot));
    pros::lcd::print(3, "Angle: %f", odomD->getState().theta.convert(degree));

    pros::Task::delay_until(&now, 10);
  }
}

bool deployed = false;

void deploy(void*params){
    LRoller.set_zero_position(0);
    LRoller.set_zero_position(0);
    LRoller.move_relative(-1500, -200);
    RRoller.move_relative(-1500, -200);
    while((LRoller.get_position()+RRoller.get_position())/2>-1000){
      pros::Task::delay(10);
    }
    deployed = true;
}




void autonomous(){
    motorInit();
    pros::Task filterT(filterPot, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Filter Potentiometer");
  	pros::Task printData(printOdom, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Odometry");


    deployed = false;
    pros::Task deployT(deploy, (void*)"PROS", TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT, "Deploy");
    driveTime(300, 400);
    driveTime(-300, 300);
    fwdD->stop(); //align against wall

    while(!deployed){
      pros::Task::delay(10);
    }

    pros::Task::delay(200);
    LRoller.move_velocity(200);
    RRoller.move_velocity(200); //rollers activate
    Lift.move_velocity(-200);

    odomD->setState({0_ft, 0_ft, 0_deg});
    slowProfile->generatePath({
      PathfinderPoint{0_ft, 0_ft, 0_deg},
      PathfinderPoint{2.25_ft, 0_ft, 0_deg}
    }, "A"); //path for 3-cube line

    slowProfile->setTarget("A");
    pros::Task::delay(500);
    Lift.move_velocity(0);

    slowProfile->waitUntilSettled(); // follow the path 3-cube
    slowProfile->flipDisable(true);

    if(isRed){
      odomD->driveToPoint(Point{0.5_ft, 2_ft}, true); // turn and drive back to .5 ft, 2 ft to the right
    } else{
      odomD->driveToPoint(Point{0.5_ft, -2_ft}, true); // turn and drive back to .5 ft, 2 ft to the left
    }
    odomD->waitUntilSettled();
    if(isRed){
      odomD->turnToPoint(Point{3_ft, 2_ft});//align to point
    } else{
      odomD->turnToPoint(Point{3_ft, -2_ft});//align to point
    }
    odomD->waitUntilSettled();

    slowProfile->generatePath({
      PathfinderPoint{0_ft, 0_ft, 0_deg},
      PathfinderPoint{3_ft, 0_ft, 0_deg}
    }, "C", slow);
    slowProfile->flipDisable(false);
    slowProfile->setTarget("C"); // set target to 4 cube line
    slowProfile->waitUntilSettled();
    slowProfile->flipDisable(true);

    if(isRed){
      odomD->driveToPoint(Point{0.75_ft, 2_ft}, true);
    } else{
      odomD->driveToPoint(Point{0.75_ft, -2_ft}, true);
    }
    odomD->waitUntilSettled();

    odomD->waitUntilSettled(); // drive to point

    LRoller.move_velocity(0);
    RRoller.move_velocity(0); //rollers deactivate
    if(isRed){
      odomD->turnToPoint(Point{-0.5_ft, 3_ft});
    } else{
      odomD->turnToPoint(Point{-0.5_ft, -3_ft});
    }
    odomD->waitUntilSettled(); // turn toward zone

    slowProfile->flipDisable(false);
    slowProfile->generatePath({
        PathfinderPoint{0_ft, 0_ft, 0_deg},
        PathfinderPoint{1_ft, 0_ft, 0_deg},
      }, "E", fast); // generate small path forward to zone

    Lift.move_velocity(-50);
    Tray.move_relative(1300, 100); // prelift
    slowProfile->setTarget("E", false, isRed);
    slowProfile->waitUntilSettled();

    while(trayRead>trayOut+50){
      if(trayRead<trayMid){
        Tray.move_velocity(100);
      } else{
        Tray.move_velocity(50+30*(double)(trayRead-trayOut)/(trayMid-trayOut));
      }
      pros::Task::delay(10);
    } //tray out with P loop
    Tray.move_velocity(0);

    LRoller.move_velocity(-100);
    RRoller.move_velocity(-100);

    driveSpeed(-500,- 600); //drive backwards*/
    fwdD->stop();
}
