const int dir = 2; //Defining dir pin (for direction) (yellow)
const int pul = 3; //Defining pul pin (for stepping) (Grey)
const int ena = 4; //Defining ena pin (for enable) (Blue)
const int relay = 10; // Defining relay pin (to turn on the the relay) // SUL EDIT (Green)

int input = 0;  //To store input recieved from bluetooth
//int prevInput;
int inputs[2] = {0, 1};

long currentPosition = 0;    //Current position in steps
long targetPosition = 0;     //Target position in steps

unsigned long stepInterval = 12; //Step interval in microseconds (calculated as 11.36, rounded to 11)
unsigned long lastStepTime = 0;    //Last time a step was taken

//For timing rotation of motor
double start = 0;
double end = 0;

bool relayFlag = false; //Goes true when relay is on // SUL EDIT

void setup()
{
  Serial.begin(9600);

  //Setting pins as OUTPUT
  pinMode(dir, OUTPUT);
  pinMode(pul, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(relay, OUTPUT); // SUL EDIT

  digitalWrite(relay, LOW); // LOW means relay is off, HIGH means relay is on // SUL EDIT
  digitalWrite(ena, HIGH);  //High enable pin means motor is disengaged
}

void loop()
{
  if (Serial.available() > 0)
  {
    //Take input on how much to press the brake
    input = Serial.read();  //Take input on how much to press the brake

    //Filter input
    char val = char(input);
    if (val == '\r' || val == '\n') 
    {
      return;
    }

    bool valid = checkinput(val, inputs); //Checking if input is valid


    // SUL CODE BEGIN
    if (val == 'a') // Stop the relay/ turn it off
    {
      relayFlag = false;
      digitalWrite(relay, LOW); 
    }
    else if (val == 'b') // Turn on the relay
    {
      relayFlag = true;
      digitalWrite(relay, HIGH); 
    }
    // SUL CODE END


    if ((valid == true) && (relayFlag == true)) // SUL EDIT
    {
      Serial.println("Turning motor on");

      // Determining direction of rotation
      if (val == '0')
      {
        //Anti-Clockwise Rotation
        digitalWrite(dir, LOW);

        //Moving motor to position specified by input
        targetPosition = 20000; //Calculating how much to rotate the motor to reach new position
        digitalWrite(ena, LOW);
        moveToPosition(targetPosition); //Rotating motor
        currentPosition = 0;
        digitalWrite(ena, HIGH);

      }
      else if (val == '1')
      {
        //Clockwise Rotation
        digitalWrite(dir, HIGH);

        //Moving motor to position specified by input
        targetPosition = 28800; //Calculating how much to rotate the motor to reach new position
        digitalWrite(ena, LOW);
        moveToPosition(targetPosition); //Rotating motor
        currentPosition = 0;
        digitalWrite(ena, HIGH);
      }
      //prevInput = val - '0';  //Updating prevInput
    }
  }
}

bool checkinput(char element, int inputs[6])
{
  //Function iterates through the list of valid inputs to check if recieved input is valid
  for (int i = 0; i < 2; i++) 
  {
    if (char(inputs[i] + '0') == element) 
    {
      return true;  //Returns true if input is valid
    }
  }
  return false; //Returns false if input is invalid   
}

void moveToPosition(long target)
{
  targetPosition = target;
  start = millis();
  Serial.println ("Stopwatch begun");
  while (currentPosition != targetPosition)
  {

    run();
    
  }
  end = millis();
  Serial.print ("Time taken (ms): ");
  Serial.println (end - start);
}

void run()
{
  unsigned long currentTime = micros();
  if (currentTime - lastStepTime >= stepInterval)
  {
    lastStepTime = currentTime;
    if (currentPosition < targetPosition)
    {
      currentPosition++;
    }
    else if (currentPosition > targetPosition)
    {
      currentPosition--;
    }
    digitalWrite(pul, HIGH);
    delayMicroseconds(9);
    digitalWrite(pul, LOW);
    delayMicroseconds(3);
  }
}
