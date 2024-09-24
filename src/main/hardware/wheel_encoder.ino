#include <Arduino.h>

int relayPin1 = 6;  // Pin connected to the first relay control input
int relayPin2 = 7;  // Pin connected to the second relay control input

volatile unsigned long pulseCount = 0;
volatile boolean counterReady = false;
volatile unsigned long overflowCount;

// Odometry variables
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double wheelBase = 1.17; // Distance between front and back wheels (in meters)
double steeringAngle = 1; // Assuming constant steering angle for now

unsigned long lastPulses = 0;
unsigned long lastTimestamp = 0;
int modeNum = 0; // This will multiply with the distance to implement gear switch's effect on distance (1 for forward, 0 for neutral, and -1 for reverse)

void startCounting() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;

  TIMSK1 = bit(TOIE1);   // interrupt on Timer 1 overflow
  TCCR2A = bit(WGM21);   // CTC mode
  OCR2A  = 124;          // count up to 125

  TIMSK2 = bit(OCIE2A);   // enable Timer2 Interrupt

  TCNT1 = 0;
  TCNT2 = 0;

  GTCCR = bit(PSRASY);  // reset prescaler now
  TCCR2B = bit(CS20) | bit(CS22);  // prescaler of 128
  TCCR1B = bit(CS10) | bit(CS11) | bit(CS12);  // External clock source on T1 pin (D5). Clock on rising edge.
}

ISR(TIMER1_OVF_vect) {
  ++overflowCount;
}

ISR(TIMER2_COMPA_vect) {
  pulseCount = (overflowCount << 16) + TCNT1;
  counterReady = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, LOW);
  startCounting();
}

void loop() {
  if (counterReady) {
    // Get current pulse count and reset for next calculation
    noInterrupts(); // We pause the ISRs just to access/overwrite data being used in them. Pending interrupts are dealt with immediately after interrupts are re enabled.
    unsigned long currentPulses = pulseCount; // Read and store the data in a stable variable. We don't want to use something that could change halfway through the loop
    counterReady = false; // Reset the counter (smothered mate)
    interrupts();

    // Get the current timestamp
    unsigned long currentTimestamp = micros();
    double deltaPulses = (currentPulses - lastPulses) * modeNum; 
    double deltaTime = (currentTimestamp - lastTimestamp) / 1000000.0; // Time difference in seconds

    // Calculate the distance traveled based on pulses
    double distance = modeNum*((31) / 740.0) * (2 * PI * 0.2032); // Assuming 740 pulses per revolution, 0.4m wheel radius

    // Update pose using the kinematic equations
    double deltaTheta = (distance / wheelBase) * tan(steeringAngle);
    x += distance * cos(theta);
    y += distance * sin(theta);
    theta += deltaTheta;

    // Print x, y, and theta to Serial
    Serial.print(x, 3);
    Serial.print(",");
    Serial.print(y, 3);
    Serial.print(",");
    Serial.print(theta, 3);
    Serial.print(",");
    Serial.print(distance, 3);
    Serial.print(",");
    Serial.println(distance/deltaTime, 3);


    // Save the current values for the next calculation
    lastPulses = currentPulses;
    lastTimestamp = currentTimestamp;

    // Delay for next loop iteration (adjust as needed)
    delay(10); // sample rate (excluding delay to execute loop)
  }

  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'f') {
      digitalWrite(relayPin2, LOW);
      digitalWrite(relayPin1, HIGH);
      modeNum = 1;
    } else if (input == 'r') {
      digitalWrite(relayPin1, LOW);
      digitalWrite(relayPin2, HIGH);
      modeNum = -1;
    } else if (input == 'n') {
      digitalWrite(relayPin1, LOW);
      digitalWrite(relayPin2, LOW);
      modeNum = 0;
    }
  }
}
