#pragma once

#include <Arduino.h>

class Ultrasonic
{
public:
    void UltraSetup();
    void UltraLoop();
    
    float average_dist;
    float median_dist;
    


private:
   
    float medianDist();
    float averageDist();
    void populateArray();
  
    

};