#include <Arduino.h>
// Include required libraries for devices
#include <Stepper.h>
#include <Ultrasonic.h>
#include <Servo.h>

/**
 * As stepper motor spins cw and ccw from 0 to 180 degrees, the
 * ultrasonic sensor takes readings at each degree and prints it
 * on the lcd. If the reading is less than some threshold, an
 * output will trigger.
*/

#define TRIGGER_DISTANCE_CM 5
#define FAN_MOTOR 13

// const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
// LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
Servo myServo;
Ultrasonic ultrasonic(6, 7); // Init ultrasonic sensor on pins 12 and 13

int distance = 9999; // Distance in cm
float angle = 0.0; // Direction angle
bool clockwise = false;
int numSteps = 0;

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
  // Set stepper speed
  myStepper.setSpeed(10);
}

// Start at 0deg and step 1 degree until 180 degrees
// After 180 degrees change direction and go back to 0
void stepperLoop()
{
  if (clockwise) // step 1/100 of a revolution clockwise
    myStepper.step(stepsPerRevolution / 100 * -1);
  else // step 1/100 of a revolution:
    myStepper.step(stepsPerRevolution / 100);

  numSteps++;

  if (numSteps > 50)
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

  pinMode(FAN_MOTOR, OUTPUT);
  myServo.attach(12);
  stepperSetup();
  ultrasonicSetup();
}

void loop()
{
  stepperLoop(); // Move motor
  ultrasonicLoop(); // Take reading
  servoLoop();

  // Process data to determine if threshold is broken
  if (distance < TRIGGER_DISTANCE_CM)
  {
    digitalWrite(FAN_MOTOR, HIGH);

    for (;;) {
      if (ultrasonic.read() > TRIGGER_DISTANCE_CM) break;
      delay(100);
    }

    digitalWrite(FAN_MOTOR, LOW);

  }
  delay(20);
}