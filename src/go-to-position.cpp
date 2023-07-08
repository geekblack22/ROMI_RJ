/*
 * Code for using TCC4 for precision PID timing.
 * You'll need to set TOP to set the interval
 * 
 * This code adds the ability to tune the gains and change the targets
 */

#include <Romi32U4.h>
#include "chassis.h"
#include "Timer.h"
#include "IRReceiver.h"
#include "IREmiter.h"
#include "Ultrasonic.h"
#include "PIDcontroller.h"



Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Motors Motors;
IRReceiver receiver;
IREmiter emit;
Ultrasonic ultra;

//const uint8_t IR_PIN = A6;


Chassis chassis;

Timer waitTimer(1000);
Timer circleTimer(10500);
float distance;
float IRerror;
float errors[20];
float position;
void setup()
{
  Serial.begin(115200);
  //while(!Serial) {}  //IF YOU DON'T COMMENT THIS OUT, YOU MUST OPEN THE SERIAL MONITOR TO START
  Serial.println("Hi");

  chassis.Init();
  waitTimer.reset();
  pinMode(18, OUTPUT);
  emit.emiterSetup();
  ultra.UltraSetup();
  receiver.receiverSetup();
}

enum STATE {IDLE, FLAT, ON_RAMP, STOPPED};
enum TYBALT {START,CIRCLE,SHOOT};
enum MERCUCIO {ROTATE,DIE};
enum ROMIFIGHT{WAIT,KILL,RUN};
STATE state = IDLE;
MERCUCIO Mstate =ROTATE;
TYBALT Tstate = START;
ROMIFIGHT rfState = WAIT;


enum DESTINATION {DEST_NONE, DEST_A, DEST_B, DEST_C};
DESTINATION destination = DEST_NONE;


void goRamp(){
  switch(state){
    case IDLE:
    if(buttonA.getSingleDebouncedPress()){
      waitTimer.reset();
    }
    if(buttonA.getSingleDebouncedRelease()){
        waitTimer.reset();
        state = FLAT;
    }
    else{
        Motors.setEfforts(0,0);
        
      }
      break;
    case FLAT:
    if(waitTimer.isExpired()){
      if(chassis.estimatedPitchAngle() <= 9 || chassis.estimatedPitchAngle() >= 13){
        Motors.setEfforts(-70,-68);
        }else{
          chassis.lightOn();
          state = ON_RAMP;
        }
  }
      
      break;
    case ON_RAMP:
      if(chassis.estimatedPitchAngle() <= 3){
        chassis.lightOff();
        state = STOPPED;
      }
      break;
      case STOPPED:
        Motors.setEfforts(0,0);
        break;
  }
}

// Fight Scene
void tybaltStates(){
  switch(Tstate){
    case START:
    emit.emit();
      circleTimer.reset();
      Tstate = CIRCLE;
      break;
    case CIRCLE:
      chassis.doCircle(45, 25);
      if(circleTimer.isExpired()){
        Tstate = SHOOT;
      }
    break;
    case SHOOT:
      chassis.SetTargetSpeeds(0,0);
      emit.stop();
    break;
    default:
    break;
  }
}

void MercucioFightStates(){
  switch (Mstate)
  {
  case ROTATE:
    
    
    chassis.rotate(IRerror, distance);
   
    break;
  
case DIE:
    chassis.lightRed();
    bool bump = distance <= 10;
    chassis.ram(IRerror,bump);
    break;
  }
}

void romiFightStates(){
  switch(rfState){
    case WAIT:
      delay(300);
      rfState = KILL;
      // if(Timer.isExpired() && DigitalRead(PIN)){
      //   rfState = KILL
      // }
    break;

    case KILL:
    break;

    case RUN:
    break;

    default:
    break;
  }
}

//Balcony Scene
void julietBalconyStates(){}
void romiBalconyStates(){}
void frairBalconyStates(){}

//Final Scene
void julietFinalStates(){}
void romiFinalStates(){}


void loop() 
{
  ultra.UltraLoop();
  distance = ultra.average_dist;
  IRerror = receiver.returnError();
  position = receiver.returnTrueX(0);
  bool bump = distance <= 4.8;
  
  // chassis.doCircle(45, 25);
  
  
  /*goRamp();
  chassis.UpdatePitch();*/

  // emit.emit();
  // tybaltStates();
  MercucioFightStates();

  if(PIDController::readyToPID) //timer flag set
  {
    // reset the flag
    PIDController::readyToPID = 0;
    
    chassis.UpdateSpeeds();
    chassis.UpdatePose();
  }

  //if(waitTimer.isExpired()) HandleTimerExpired();
  
}