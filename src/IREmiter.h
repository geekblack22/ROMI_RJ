#pragma once

#include <Arduino.h>
#include <Romi32U4.h>
#include <Timer.h>
class IREmiter{
    private: 
        const int emiter = 11;
        bool irOn = false;
       
    public:
       void emit();
       void stop();
       void emiterSetup();
};