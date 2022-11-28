#include <Arduino.h>
// Include required libraries for devices
#include <Stepper.h>
#include <Ultrasonic.h>
#include <Servo.h>
#include <Encoder.h>
#include <EEPROM.h>

/**
 * As stepper motor spins cw and ccw from 0 to 180 degrees, the
 * ultrasonic sensor takes readings at each degree and prints it
 * on the lcd. If the reading is less than some threshold, an
 * output will trigger.
*/

// Uncomment the stepper motor being used
#define NEMA17 1
// #define STARTER_STEPPER 1

#define TRIGGER_DISTANCE_CM 5

#define FAN_MOTOR 10
#define BEEPER_PIN 11
#define ENCODER_BUTTON 4

#define BLUE_LED A3
#define RED_LED A2

// const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
// LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
Stepper *myStepper;
Servo myServo;
Ultrasonic ultrasonic(A5, A4); // Init ultrasonic sensor on pins 12 and 13
Encoder myEnc(2, 3);

long oldPosition = -999;

int distance = 9999; // Distance in cm
float angle = 0.0; // Direction angle
bool clockwise = false;
int numSteps = 0;

int stepRange = 40; // Number of steps to take for each scan pass

/*
// Init lcd and any variables needed
void lcdSetup()
{
  lcd.begin(16, 2); // Start lcd with 16 columns and 2 rows

  lcd.setCursor(0, 0);
  lcd.print("Angle: ");
  lcd.print("Distance: ");
}

// Log current read angle and distance from ultrasonic sensor
void lcdLoop()
{
  // Print the current angle on the first row after the text
  lcd.setCursor(0, 8);
  lcd.print(angle, 2);

  // Print the current read distance on the second row after the text
  lcd.setCursor(0, 11);
  lcd.print(distance);
  lcd.print(" CM");
}
*/

void stepperSetup()
{
#ifdef NEMA17
  // Set stepper speed
  stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
  myStepper = new Stepper(stepsPerRevolution, 6, 7, 8, 9);
  myStepper->setSpeed(10);
#endif

#ifdef STARTER_STEPPER
  stepsPerRevolution = 4096;  // change this to fit the number of steps per revolution
  myStepper = new Stepper(stepsPerRevolution, 6, 7, 8, 9);
  myStepper->setSpeed(10);
#endif
}

// Start at 0deg and step 1 degree until 180 degrees
// After 180 degrees change direction and go back to 0
void stepperLoop()
{
  if (clockwise) // step 1/100 of a revolution clockwise
    myStepper->step(stepsPerRevolution / 100 * -1);
  else // step 1/100 of a revolution:
    myStepper->step(stepsPerRevolution / 100);

  numSteps++;

  if (numSteps > stepRange)
  {
    clockwise = !clockwise;
    numSteps = 0;
  }
}

void ultrasonicSetup()
{
  // Not much needed for now
  // distance = 1000; (?)
}

void encoderSetup()
{
  tone(BEEPER_PIN, 1000, 75);
  delay(150);
  tone(BEEPER_PIN, 1000, 75);
  delay(75);

  oldPosition = myEnc.read();

  // Set negative bound
  while (true)
  {
    long newPosition = myEnc.read();
    if ((newPosition - oldPosition) >= 4)
    {
      myStepper->step(stepsPerRevolution / 100);
      oldPosition = newPosition;
    }
    if ((newPosition - oldPosition) <= -4)
    {
      myStepper->step(stepsPerRevolution / 100 * -1);
      oldPosition = newPosition;
    }

    if (digitalRead(ENCODER_BUTTON) == LOW) break;
  }

  tone(BEEPER_PIN, 1000, 150);
  for (int i=0; i<stepRange; i++)
  {
    myStepper->step(stepsPerRevolution / 100 * -1);
  }

  while (true)
  {
    long newPosition = myEnc.read();
    if ((newPosition - oldPosition) >= 4)
    {
      myStepper->step(stepsPerRevolution / 100);
      stepRange -= 1;
      oldPosition = newPosition;
    }
    if ((newPosition - oldPosition) <= -4)
    {
      myStepper->step(stepsPerRevolution / 100 * -1);
      stepRange += 1;
      oldPosition = newPosition;
    }

    if (digitalRead(ENCODER_BUTTON) == LOW) break;
  }

  EEPROM.write(1, stepRange);

  tone(BEEPER_PIN, 1300, 100);
  delay(100);
  tone(BEEPER_PIN, 1500, 100);
  delay(100);

  // while (digitalRead(ENCODER_BUTTON)) { delay(10); }

  // tone(BEEPER_PIN, 1500, 75);
}

void ultrasonicLoop()
{
  distance = ultrasonic.read();
  Serial.println(distance);
}

void servoLoop()
{
  myServo.write(
    map(distance, 0, 100, 0, 500)
  );
}

void setup()
{
  Serial.begin(9600);
  EEPROM.begin();

  stepRange = EEPROM.read(1);

  pinMode(FAN_MOTOR, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);

  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  myServo.attach(12);
  stepperSetup();
  ultrasonicSetup();

  encoderSetup();
}

void loop()
{
  stepperLoop(); // Move motor
  ultrasonicLoop(); // Take reading
  servoLoop();

  // Process data to determine if threshold is broken
  if (distance < TRIGGER_DISTANCE_CM)
  {
    bool beep = false;
    digitalWrite(FAN_MOTOR, HIGH);

    for (;;) {
      if (ultrasonic.read() > TRIGGER_DISTANCE_CM) break;

      if (beep)
      {
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(RED_LED, LOW);
        tone(BEEPER_PIN, 2000, 100);
      }
      else {
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
      }
      beep = !beep;
      delay(100);
    }

    digitalWrite(FAN_MOTOR, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, LOW);

  }
  delay(20);
}