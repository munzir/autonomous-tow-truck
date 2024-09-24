#include "brake.h"

//Function to initialize the brake system
void brake_setup(struct Brake* brake) 
{
  pinMode(brake->dir, OUTPUT);
  pinMode(brake->pul, OUTPUT);
  pinMode(brake->ena, OUTPUT);
  pinMode(brake->relay, OUTPUT);

  digitalWrite(brake->relay, LOW);  //LOW means relay is off, HIGH means relay is on
  digitalWrite(brake->ena, HIGH); //High enable pin means motor is disengaged

}

//Function to check if input is valid
bool check_input(char element, struct Brake* brake) 
{
  //Function iterates through the list of valid inputs to check if recieved input is valid
  for (int i = 0; i < 2; i++) 
  {
    if ((char)(brake->inputs[i] + '0') == element) 
    {
      return true;  //Returns true if input is valid
    }
  }
  return false; //Returns false if input is invalid
}

//Function to turn relay on
void relay_control_on(struct Brake* brake)
{
  brake->relayFlag = true;
  digitalWrite(brake->relay, HIGH);
}

//Function to turn relay off
void relay_control_off(struct Brake* brake)
{
  brake->relayFlag = false;
  digitalWrite(brake->relay, LOW);
}

//Function to press brake
void press_brake(struct Brake* brake)
{
  digitalWrite(brake->dir, HIGH);  
  brake->targetPosition = 28800;
  digitalWrite(brake->ena, LOW);
  move_to_position(brake, brake->targetPosition);
  brake->currentPosition = 0;
  digitalWrite(brake->ena, HIGH);
}

//Function to release brake
void release_brake(struct Brake* brake)
{
  digitalWrite(brake->dir, LOW);  
  brake->targetPosition = 20000;
  digitalWrite(brake->ena, LOW);
  move_to_position(brake, brake->targetPosition);
  brake->currentPosition = 0;
  digitalWrite(brake->ena, HIGH);
}

//Function to move the motor to specified position
void move_to_position(struct Brake* brake, long target) 
{
  brake->targetPosition = target;
  while (brake->currentPosition != brake->targetPosition) 
  {
    run(brake);
  } 
}

//Function to rotate the motor
void run(struct Brake* brake) 
{
  unsigned long currentTime = micros();
  if (currentTime - brake->lastStepTime >= brake->stepInterval) 
  {
    brake->lastStepTime = currentTime;
    if (brake->currentPosition < brake->targetPosition) 
    {
      brake->currentPosition++;
    } else if (brake->currentPosition > brake->targetPosition) 
    {
      brake->currentPosition--;
    }
    digitalWrite(brake->pul, HIGH);
    delayMicroseconds(9);
    digitalWrite(brake->pul, LOW);
    delayMicroseconds(3);
  }
}