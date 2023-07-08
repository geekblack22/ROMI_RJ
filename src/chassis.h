#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <Romi32U4.h>
#include "PIDcontroller.h"

class Chassis
{
private:
    //parameters -- these will need to be updated after you do your experiments
    float wheel_track = 15.21; //cm
    float wheel_diam = 7.0; //cm
    float ticks_per_rotation = 1440; // from the datasheet

    //current Pose
    float x = 0;
    float y = 0;
    float theta = 0;

    //current target
    float x_target = 0;
    float y_target = 0;
    float th_target = 0; //may or may not be specified

    //target wheels speeds
    float targetSpeedLeft = 0;
    float targetSpeedRight = 0;
    const float countsPerCM = 65.75;
    //for calculating speeds
    int16_t prevEncLeft = 0;
    int16_t prevEncRight = 0;

    //wheel speed controllers
    PIDController leftMotorController; 
    PIDController rightMotorController; 

    //actual speed
    int16_t speedLeft = 0;
    int16_t speedRight = 0;

    //be sure to set this -- it needs to match your "readyToPID" period
    uint32_t timestepMS = 16; // in ms

    //Observed angle from acceleromitor
    float obsAngleAcc = 0;

    //Predicted Angle from gyroscope
    float preAngleGyro = 0;

    int count = 0;
    float x_off;
    float cal_sum;

public:
    bool finishedCircle = false;
    bool finishedRotation = false;
    Chassis(void);
    void Init(void);
    void SetTargetSpeeds(float left, float right) { targetSpeedLeft = left; targetSpeedRight = right; }
    void SetTargetPosition(float xt, float yt) { x_target = xt; y_target = yt; }
    void UpdatePose(void);
    void UpdateSpeeds(void);
    void MoveToPoint(float x, float y);
    bool atDestination();
    bool UpdatePitch();
    float estimatedPitchAngle();
    float accXoffset();
    float gyroBias();
    bool calibration = false;
    void lightOn();
    void lightRed();
    void lightBlue();
    void lightOff();
    void ram(float error, bool stop);
    void rotate(float error, float position);
    void doCircle(float radius, float u0);
    void positionController();
};

#endif