#include "Ultrasonic.h"
volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;

//define the states for the echo capture
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};

//and initialize to IDLE
volatile PULSE_STATE pulseState = PLS_IDLE;

//this may be most any pin, connect the pin to Trig on the sensor
const uint8_t trigPin = 14;

//for scheduling pings
uint32_t lastPing = 0;
const uint32_t PING_INTERVAL = 100; //ms
float five_val[5];
int dist_count;
float distanceCalc;

/*
 * Commands the ultrasonic to take a reading
 */
void CommandPing(int trigPin)
{
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;
  
  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //we'll allow a delay here for convenience; it's only 10 us
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

// updates array with 5 most recent distance readings
void Ultrasonic:: populateArray(){
  
  five_val[dist_count] = distanceCalc;
}

// updates array into a increasing order
void orderArray(){
  float temp;
  for(int i = 0; i < 5; i++) {
    for(int j = i+1;j < 5; j++){
      if(five_val[j] < five_val[i]){
      temp = five_val[i];
      five_val[i] = five_val[j];
      five_val[j] = temp;
      }
    }
  }
}


// return a float value of the median of 5 distance readings
float Ultrasonic ::medianDist(){
  return five_val[2];
}

// return a float value of the average of 5 distance readings
float Ultrasonic ::averageDist(){
  float sum = five_val[0] + five_val[1] + five_val[2] + five_val[3] + five_val[4];
  return sum / 5.0;
}

void Ultrasonic::UltraSetup()
{
 
  
  Serial.println("setup");

  noInterrupts(); //disable interupts while we mess with the control registers
  
  //sets timer 3 to normal mode (16-bit, fast counter)
  TCCR3A = 0; 
  
  interrupts(); //re-enable interrupts

  //note that the Arduino machinery has already set the prescaler elsewhere
  //so we'll print out the value of the register to figure out what it is
  Serial.print("TCCR3B = ");
  Serial.println(TCCR3B, HEX);

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)

  lastPing = millis();

  Serial.println("/setup");
}


void Ultrasonic::UltraLoop() 
{
  //schedule pings roughly every PING_INTERVAL milliseconds
  
  uint32_t currTime = millis();
  populateArray();
  
  dist_count++;
  
  if(dist_count == 5){
    
    dist_count = 0;
    orderArray();
    if( averageDist() > .5){
    average_dist = averageDist();
    }
    if(median_dist > .5){
    median_dist = medianDist(); 
    }
   
  }
   
   
  if((currTime - lastPing) >= PING_INTERVAL && pulseState == PLS_IDLE)
  {
    lastPing = currTime;
    CommandPing(trigPin); //command a ping
  }
  
  if(pulseState == PLS_CAPTURED) //we got an echo
  {
    //update the state to IDLE
    pulseState = PLS_IDLE;

    /*
     * Calculate the length of the pulse (in timer counts!). Note that we turn off
     * interrupts for a VERY short period so that there is no risk of the ISR changing
     * pulseEnd or pulseStart. The way the state machine works, this wouldn't 
     * really be a problem, but best practice is to ensure that no side effects can occur.
     */
    noInterrupts();
    uint16_t pulseLengthTimerCounts = pulseEnd - pulseStart;
    interrupts();
    
    //EDIT THIS LINE: convert pulseLengthTimerCounts, which is in timer counts, to time, in us
    //You'll need the clock frequency and the pre-scaler to convert timer counts to time
    
    uint32_t pulseLengthUS = pulseLengthTimerCounts * 4;  //pulse length in us


    //EDIT THIS LINE AFTER YOU CALIBRATE THE SENSOR: put your formula in for converting us -> cm
    float distancePulse = ((343.0*100.0*pulseLengthUS)/2000000);    //distance in cm

    //Inversion Equation for distance
    distanceCalc = (pulseLengthUS + 0.0168) / 58.309;

    /*Serial.print(millis());
    Serial.print('\t');
    Serial.print(pulseLengthTimerCounts);
    Serial.print('\t'); */
    /*Serial.print("Raw: ");
    Serial.print(distanceCalc);
    Serial.print("\t");
    Serial.print("Average: ");
    Serial.print(average_dist);
    Serial.print("\t");
    Serial.print("Median :");
    Serial.println(median_dist);*/
   
    
    /*
    Serial.print(pulseLengthUS);
    Serial.print('\t');
    Serial.print(distancePulse);
    Serial.print('\t');
    Serial.print(distanceCalc);
    Serial.print('\n');*/
  }
  
}

/*
 * ISR for input capture on pin 13. We can precisely capture the value of TIMER3
 * by setting TCCR3B to capture either a rising or falling edge. This ISR
 * then reads the captured value (stored in ICR3) and copies it to the appropriate
 * variable.
 */
ISR(TIMER3_CAPT_vect)
{
  if(pulseState == PLS_WAITING_LOW) //we're waiting for a rising edge
  {
    pulseStart = ICR3; //copy the input capture register (timer count)
    TCCR3B &= 0xBF;    //now set to capture falling edge on pin 13
    pulseState = PLS_WAITING_HIGH;
  }

  else if(pulseState == PLS_WAITING_HIGH) //waiting for the falling edge
  {
    pulseEnd = ICR3;
    pulseState = PLS_CAPTURED; //raise a flag to indicate that we have data
  }
  
}