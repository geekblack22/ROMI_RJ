#pragma once

#include <Arduino.h>
#include <IRDirectionFinder.h>
#include <Wire.h>


class IRReceiver{
    private:
        float band = 100.0;

    public:
        void receiverSetup();
        int returnX(int point);
        int returnTrueX(int point);
        int returnY(int point);
        bool centered();
        int returnError();
        bool seeIR();
};