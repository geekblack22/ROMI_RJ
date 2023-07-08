#include "IRReceiver.h"

IRDirectionFinder irFinder;

void IRReceiver::receiverSetup()
{
  irFinder.begin();
}

int IRReceiver::returnX(int i){
    static int x_val = 0;
    irFinder.requestPosition();
    if(irFinder.available()){
        Point point = irFinder.ReadPoint(i);
        if(point.x != 1023){
            x_val = point.x;
        }
    }
    return x_val;
}

int IRReceiver::returnTrueX(int i){
    static int x_val = 0;
    irFinder.requestPosition();
    if(irFinder.available()){
        Point point = irFinder.ReadPoint(i);
        x_val = point.x;
    }
    return x_val;
}

int IRReceiver::returnY(int i) {
    static int y_val = 0;
    irFinder.requestPosition();
    if(irFinder.available()){
        irFinder.requestPosition();
        Point point = irFinder.ReadPoint(i);
        if(point.y != 1023) {
            y_val = point.y;
        }
    }
    return y_val;
}

bool IRReceiver::centered(){
    return (returnX(0) > (500 - (band)) && returnX(0) < (500 + (band)));
}

int IRReceiver::returnError(){
    static int x_val = 0;
    irFinder.requestPosition();
    if(irFinder.available()){
        Point point = irFinder.ReadPoint(0); 
        if(point.x != 1023){
            x_val = point.x;
        }
    }
    int error = 500 - x_val;
    return error;
}

bool IRReceiver::seeIR(){
    irFinder.requestPosition();
    if(irFinder.available()){
        Point point = irFinder.ReadPoint(0);
        return point.x != 1023;
    }
}