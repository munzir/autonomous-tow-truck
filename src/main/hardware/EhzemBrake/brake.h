#ifndef BRAKE_H
#define BRAKE_H

#include <Arduino.h>

//Struct to encapsulate brake data and functionality
struct Brake 
{
  //Pin Definiations
  const int dir;  //Direction pin (yellow)
  const int pul;  //Pulse pin  (grey)
  const int ena;  //Enable pin (blue)
  const int relay;  //Relay pin (to turn on the the relay) (green)

  //Variables
  long currentPosition; //Current position in steps
  long targetPosition;  //Target position in steps
  unsigned long stepInterval; //Step interval in microseconds
  unsigned long lastStepTime; //Last time a step was taken
  bool relayFlag; //Goes true when relay is on
  int inputs[2];  //Allowed inputs for validation
};

//Function prototypes
void brake_setup(struct Brake* brake);
bool check_input(char element, struct Brake* brake);  
void relay_control_on(struct Brake* brake);
void relay_control_off(struct Brake* brake);
void press_brake(struct Brake* brake);
void release_brake(struct Brake* brake);
void move_to_position(struct Brake* brake, long target);
void run(struct Brake* brake);


#endif  // BRAKE_H
