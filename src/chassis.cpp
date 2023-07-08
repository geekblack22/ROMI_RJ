#include "chassis.h"
#include <Timer.h>


#define FASTLED_INTERRUPT_RETRY_COUNT 0
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <FastLED.h>
#define NUM_LEDS 26
CRGB leds[NUM_LEDS];
const int ledPin = 18;
const int LimitSwitchPin = 20;

#include <math.h>

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

bool showAcc = false;
bool showGyro = true;


float threshold = 5.0;

Romi32U4Encoders encoders;
Romi32U4Motors motors;
PIDController distancePID = PIDController(1.25);
PIDController thetaPID = PIDController(70);
PIDController ramPID = PIDController(.1);
PIDController rotatePID = PIDController(.1);
PIDController irPID = PIDController(0.25, 0.01, 0.0, 5000);

LSM6 imu;
enum CIRCLESTATE {ARC1, ARC2, ARC3,ARC4};
CIRCLESTATE Cstate = ARC1;

Chassis::Chassis(void) 
    //we'll use the member initializer capabilities of C++ to set up the controllers
    : leftMotorController(5, 1, 0, 500), rightMotorController(5, 1, 0, 500)  
    {
        SetTargetSpeeds(0, 0);
    }

void Chassis::Init(void)
{  
    noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
    //sets up timer 4
    TCCR4A = 0x00; //disable some functionality -- no need to worry about this
    TCCR4B = 0x0B; //sets the prescaler -- look in the handout for values
    TCCR4C = 0x04; //toggles pin 6 at the timer frequency
    TCCR4D = 0x00; //normal mode

    /*
    * EDIT THE LINE BELOW WITH YOUR VALUE FOR TOP
    */

    OCR4C = 249;   //TOP goes in OCR4C 
    timestepMS = 16; //should correspond to your choice for TOP

    TIMSK4 = 0x04; //enable overflow interrupt

    interrupts(); //re-enable interrupts

    //pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!


    Wire.begin();

    if (!imu.init())
    {
        // Failed to detect the LSM6.
        ledRed(1);
        while(1)
        {   
        Serial.println(F("Failed to detect the LSM6."));
        delay(100);
        }
    }

    imu.enableDefault();

    // Set the gyro full scale and data rate
    imu.setGyroDataOutputRate(LSM6::ODR13);

    // Set the accelerometer full scale and data rate
    imu.setAccDataOutputRate(LSM6::ODR13);

    FastLED.addLeds<WS2812B, ledPin, GRB>(leds, NUM_LEDS);
}

void Chassis::lightOn(){
    for(int i = 0; i <(NUM_LEDS); i++){
    leds[i] = CRGB::Magenta;
    }
  FastLED.show();
}

void Chassis::lightRed(){
    for(int i = 0; i <(NUM_LEDS); i++){
    leds[i] = CRGB::Red;
    }
  FastLED.show();
}

void Chassis::lightBlue(){
    for(int i = 0; i <(NUM_LEDS); i++){
    leds[i] = CRGB::Blue;
    }
  FastLED.show();
}

void Chassis::lightOff(){
    for(int i = 0; i <(NUM_LEDS); i++){
    leds[i] = CRGB::Black;
    }
  FastLED.show();
}

void Chassis::UpdatePose(void)
{
    

    //conversion from ticks/interval to cm/sec
    float conversionToCMPerSec = 1000.0/(16.0*countsPerCM); //YOU'LL NEED TO CALCULATE THIS VALUE
    
    float spLeft = speedLeft * conversionToCMPerSec;
    float spRight = speedRight * conversionToCMPerSec;

    //average speed
    float u_0 = (speedRight+speedRight)/2.0; //YOU'LL NEED TO ADD THIS EXPRESSION

    //omega
    float omega = (spRight - spLeft)/wheel_track; //YOU'LL NEED TO ADD THIS EXPRESSION

    //simple first-order method -- not sufficient for class
    float dt = timestepMS / 1000.00; //SET timestepMS IN THE CONSTRUCTOR

    float spDiff = spRight - spLeft;
    float spSum = spRight + spLeft;

    float r = (wheel_track/2.0)*(spSum/spDiff);

    //YOU'LL NEED TO CALCULATE THESE
    if(abs(speedRight-spLeft) >= 3.0){
    
        theta += omega*dt;
        float xtheta = r*(sin(theta+(omega*dt))-sin(theta));
        float ytheta = r*(cos(theta)-cos(theta+(omega*dt)));
        if((float)xtheta == xtheta) {
            x += xtheta;
        }
        if((float)ytheta == ytheta) {
            y += ytheta;
        }
        //x += r*(sin(theta+(omega*dt))-sin(theta));
        //y += r*(cos(theta)-cos(theta+(omega*dt)));
        //Serial.println(x);
    }
    else{
        x += u_0*cos(theta)*dt;
        y += u_0*sin(theta)*dt;
       theta += 0; 
       //Serial.println("No turn");
    }    
}

bool Chassis::atDestination(){
    float x_error = x_target - x;
    float y_error = y_target - y;
    if(abs(x_error) < threshold && abs(y_error) < threshold){
        return true;
    }
    else return false;
}

void Chassis::positionController(){
    /*
     * Do PID stuffs here. Note that we turn off interrupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
     */
    float distance_error = sqrt(square(x_target - x) + square(y_target - y));
    float theta_error = 0;
    
        th_target = atan2(y_target - y,x_target - x);
        theta_error = th_target - theta;
    
    
    float thetaTerm = thetaPID.ComputeEffort(theta_error);
    float distanceTerm = distancePID.ComputeEffort(distance_error);
    float speedL = distanceTerm - thetaTerm;
    float speedR = distanceTerm + thetaTerm;
   
    
    SetTargetSpeeds(speedL,speedR);
}

void Chassis::UpdateSpeeds(void)
{        
    noInterrupts();

    speedLeft = countsLeft - prevEncLeft;
    prevEncLeft = countsLeft;
    speedRight = countsRight - prevEncRight;
    prevEncRight = countsRight;
   
    interrupts();
    
   //float target_distance = sqrt(square(x_target) + square(y_target));
   //float distance;  //=sqrt(square(x_target - x) + square(y_target - y))
    
        

    int16_t errorLeft = targetSpeedLeft - speedLeft; //calculate the error
    float effortLeft = leftMotorController.ComputeEffort(errorLeft); //calculate effort from error
    

    int16_t errorRight = targetSpeedRight - speedRight; //calculate the error
    float effortRight = rightMotorController.ComputeEffort(errorRight); //calculate effort from error
    
    motors.setEfforts(effortLeft,effortRight); 
    // Serial.print("\t");   
}


void Chassis::MoveToPoint(float x, float y){
    SetTargetPosition(x,y);
}
/*
 * ISR for timing. On overflow, it takes a 'snapshot' of the encoder counts and raises a flag to let
 * the main program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  PIDController::readyToPID = 1;
}

bool Chassis::UpdatePitch(){
    
    boolean newReading = imu.getStatus() & 0x01;
    static float previousAngle = 0;
    float gyroReading = 0;

  if(newReading)
  {
    imu.read();



    //Calculating angle from acc
    obsAngleAcc = (180/2*PI)*atan2(-imu.a.x,imu.a.y);

    //calculating angle from gyro
    gyroReading = imu.g.y + 567.7747748;
    preAngleGyro = previousAngle + ((1/13.0)*(gyroReading)/8.75);
    

    if(showAcc)
    {
      Serial.print(imu.a.x);
      Serial.print(' ');
      Serial.print(imu.a.y);
      Serial.print(' ');
      Serial.print(imu.a.z);
      Serial.print(' ');
      Serial.print(obsAngleAcc);
      Serial.print(' ');
      Serial.print(preAngleGyro);
      Serial.print(' ');


   
  }if(showGyro)
    {
      
      Serial.print(estimatedPitchAngle());
      Serial.print(' ');
    }
     Serial.print('\n');
    previousAngle = preAngleGyro;
    return newReading;
    
}
}
float Chassis::accXoffset(){
    
if(calibration){
    cal_sum += imu.a.x;
    count++;
}

if(count >= 200){
    x_off = cal_sum/200.0;
    calibration = false;
    count = 0;
    cal_sum = 0;
    return x_off;
}
}

float Chassis::gyroBias(){
       
    if(calibration){
        cal_sum += imu.g.y;
        count++;
    }

    if(count >= 200){
        x_off = cal_sum/200.0;
        calibration = false;
        count = 0;
        cal_sum = 0;
        return x_off;
    }
}

float Chassis::estimatedPitchAngle(){
    float k = .015;
    float estimatedAngle = 0;
    
    //gyro
    static float previousAngle = 0;
    float gyroReading = 0;
    //calculating angle from gyro
    gyroReading = imu.g.y + 567.7747748;
    preAngleGyro = (previousAngle + ((1/13.0)*(gyroReading)*8.75)/1000.0);
    previousAngle = preAngleGyro;

    //acc
    //Calculating angle from acc
    obsAngleAcc = (180/PI)*atan2(-imu.a.x,imu.a.z);

    //combo
    estimatedAngle = (k*preAngleGyro) + (1-k)*obsAngleAcc;
    return estimatedAngle;
}


void Chassis::ram(float error, boolean stop){
    
    float effort;
    float left;
    float right;
    if(stop){
        left = 0;
        right = 0;
        lightRed();
    }else{
        effort = ramPID.ComputeEffort(error);
       left = effort + 100;
       right = 100 - effort;
        lightOff();
    }
    motors.setEfforts(left, right);
    
}

void Chassis::rotate(float error, float position){
    float effort = irPID.ComputeEffort(error);
    float left;
    float right;
    static int counter;
    static float previous_error;
     if(position <= 10.0){
        
       ++counter;
     }
     if(counter <= 3){
        left = effort;
        right = -effort;
     }
     else{
        left = 0;
        right = 0;
        finishedRotation = true; 
     } 
      motors.setEfforts(left, right);
      previous_error = error;
      Serial.println(error);
}

void Chassis::doCircle(float radius, float u0){
    float speedDiff =(wheel_track * u0) / (2.0 * radius);
    int16_t ur = u0 + speedDiff;
    int16_t ul = u0 - speedDiff;
    SetTargetSpeeds(ul, ur);
}