#include "IREmiter.h"

Timer wait(250);
Romi32U4Motors motors2;
void IREmiter::emiterSetup(){
  pinMode(emiter, OUTPUT);
  wait.reset();
  motors2.init();

}

void IREmiter::emit(){
    if(wait.isExpired()){
     if(!irOn){
      OCR1C = 420;
      irOn = true;
    }
    else if(irOn){
      OCR1C = 0;
      irOn = false;
    }
    wait.reset();
  }
}

void IREmiter::stop(){
  OCR1C = 0;
}