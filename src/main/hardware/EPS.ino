// Control of EPS motor 
// Input is Duty cycle which varies from -255 to 255, beyond those values it stops. 0 is unavailable for use
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
int inputV = 0;
void setup()
{
  Serial.begin(9600);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
}
 
void loop()
{
  // analogWrite(RPWM_Output, 100);
  // digitalWrite(LPWM_Output, LOW);
  if (Serial.available() > 0)
  {
    // Take in input
    inputV = Serial.parseInt();

    // value is in the range -255 to 255
    // the lower half of it we use for reverse rotation; the upper half for forward rotation
    if (inputV != 0) // randomly throws a 0, this line stops it
    {
      Serial.println(inputV);  
      if ((inputV < -255) || (inputV > 255))
      {
        Serial.println("STOP");
        digitalWrite(LPWM_Output, 0);
        digitalWrite(RPWM_Output, 0);
      }
      else if (inputV < 0)
      {
        // reverse rotation
        Serial.println("reverse");
        int reversePWM = -inputV;
        digitalWrite(LPWM_Output, 0);
        analogWrite(RPWM_Output, reversePWM);
      }
      else
      {
        // forward rotation
        Serial.println("frwrd");
        int forwardPWM = inputV;
        digitalWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, forwardPWM);
      }
    }
  }
}
