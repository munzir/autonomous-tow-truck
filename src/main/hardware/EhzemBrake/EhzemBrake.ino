#include "brake.h"

//Global brake instance
struct Brake brake = {
  .dir = 2,
  .pul = 3,
  .ena = 4,
  .relay = 10,
  .currentPosition = 0,
  .targetPosition = 0,
  .stepInterval = 12,
  .lastStepTime = 0,
  .relayFlag = false,
  .inputs = {0, 1}
};


void setup() 
{
  Serial.begin(9600);
  brake_setup(&brake);  
}

void loop() 
{
  if (Serial.available() > 0) 
  {
    int input = Serial.read();  //To store input recieved from bluetooth
    
    // Filter input
    char val = (char)input;
    if (val == '\r' || val == '\n') 
    {
      return;
    }

    if (input == 'a') //Turn relay off
    {
      Serial.println("OFF");
      relay_control_off(&brake);
    }
    else if (input == 'b')  //Turn relay on
    {
      Serial.println("ON");
      relay_control_on(&brake);
    }

    if (check_input(val, &brake)) //Check if the input is valid
    {
      //When system is in TeleOp mode Electronically
      if (brake.relayFlag) 
      {
        //Determine direction of rotation
        if (input == '0') //Releasing Brake
        {
          release_brake(&brake);
        }
        else if (input == '1') //Pressing Brake
        {
          press_brake(&brake);
        }
      }
    }
  }
}
