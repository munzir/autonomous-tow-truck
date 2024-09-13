// Initial Code Push
// Works on sending 0-5V on serial, -1 sets teleop control, -2 sets manual control
char command;
int pin = 9;
float receive;
int idle = 11;
float lastValue = 0;  // Variable to store the last received value
int flag = 0; // 0 means controlled using the pedal, 1 means control using Arduino
int flagpin = 6;
// FIRST SET FLAG TO BE HIGH BY SENDING -1, THEN SEND VOLTAGES. SET FLAG TO BE 0 BY SENDING -2
void setup()
{
  Serial.begin(9600);
  pinMode(pin, OUTPUT);
  pinMode(idle, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0)
  {
    // Take in input
    receive = Serial.parseFloat();
    Serial.println(receive);
    
    // Randomly sends 0, ignore it
    // If 1 sent, stop, else set duty cycle
    if (receive == -2) // MANUAL MODE
    {
      analogWrite(pin, 0);
      analogWrite(idle, 0);
      lastValue = 0;
      flag = 0;
      analogWrite(flagpin, 0); // INTERNAL FLAG
    }
    else if (receive == -1) // SWITCH TO TELEOP MODE
    {
      Serial.println("here");

      analogWrite(pin, 0);
      analogWrite(idle, 0);
      lastValue = 0;
      flag = 1;
      analogWrite(flagpin, 255);
    }
    else if (receive != 0.00)
    {
      lastValue = receive;  // Update the last value if it's not 0
    }
  }
  
    // Write the last received value
    if (flag == 1) // EXTRA CHECK
    {
      if (lastValue == 0.00)
      {
        analogWrite(pin, 0);
        analogWrite(idle, 0);
      }
      else
      {
        analogWrite(pin, int(255 / 5 * (lastValue )));
        Serial.println(int(255 / 5 * (lastValue )));
        analogWrite(idle, 255);
      }
    }
}
