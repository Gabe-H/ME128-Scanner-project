#include <Arduino.h>
// Include required libraries for devices
#include <Stepper.h>
#include <Ultrasonic.h>
#include <Servo.h>
#include <Encoder.h>
#include <EEPROM.h>

#include <IRremote.h>
#include <ir_defs.h>

/**
 * As stepper motor spins cw and ccw from 0 to x degrees, the
 * ultrasonic sensor takes readings at each 1/100th degree.
 * If the reading is less than some threshold, a buzzer
 * will beep for some time, then leds will flash and a motor
 * is triggered.
*/

// Uncomment the stepper motor being used
#define NEMA17 1
// #define STARTER_STEPPER 1

#define TRIGGER_DISTANCE_CM 30 // Ultrasonic sensor threshold

// Pin configuration
#define FAN_MOTOR A4
#define BEEPER_PIN A3
#define ENCODER_BUTTON 7

#define BLUE_LED A2
#define RED_LED A1

#define SERVO_PIN 12

#define IR_PIN 6

Encoder myEnc(5, 4);
Ultrasonic ultrasonic(3, 2);
IRrecv ir(IR_PIN);
////////////////////

Stepper *myStepper;
Servo myServo;

// change this to fit the number of steps per revolution
int stepsPerRevolution = 200;

int distance = 9999; // Distance in cm
float angle = 0.0; // Direction angle
bool clockwise = false;
int numSteps = 0;
int stepRange = 40; // Number of steps to take for each scan pass

long oldPosition = -999; // Old encoder position tracker

bool running = true;


void stepperSetup()
{
  // Set stepper speed - change this to fit the number of steps per revolution
  stepsPerRevolution = 200;
  myStepper = new Stepper(stepsPerRevolution, 8, 9, 10, 11);
  myStepper->setSpeed(10);
}

// Start at 0deg and step 1/100 rev, until limit reached, then turn around
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

void irLoop()
{
  if (!ir.decode()) return;

  uint16_t cmd = ir.decodedIRData.command;

  switch (cmd)
  {
  case IR_POWER:
    running = false;
    break;
  case IR_PLAY_PAUSE:
    running = !running;
    tone(BEEPER_PIN, 1500, 100);
    break;
  case IR_UP:

    break;
  case IR_DOWN:

    break;

  default:
    // Unknown command
    Serial.print("Unknown button pressed: ");
    Serial.println(cmd);
    break;
  }
}

void encoderSetup()
{
  // Startup beeps
  tone(BEEPER_PIN, 1000, 75);
  delay(150);
  tone(BEEPER_PIN, 1000, 75);

  // myStepper->setSpeed(5);

  oldPosition = myEnc.read(); // Init encoder position

  // Set negative bound setup
  while (true)
  {
    long newPosition = myEnc.read();
    if ((newPosition - oldPosition) >= 4) // Encoder turned clockwise
    {
      myStepper->step(stepsPerRevolution / 100);
      oldPosition = newPosition;
    }
    if ((newPosition - oldPosition) <= -4) // Encoder turned ccw
    {
      myStepper->step(stepsPerRevolution / 100 * -1);
      oldPosition = newPosition;
    }

    if (digitalRead(ENCODER_BUTTON) == LOW) break;
  }

  tone(BEEPER_PIN, 1000, 150);
  // Move to configured bound
  for (int i=0; i<stepRange; i++)
  {
    myStepper->step(stepsPerRevolution / 100 * -1);
  }

  // Same setup as before, but track how many steps taken
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

  // myStepper->setSpeed(10);

  EEPROM.write(1, stepRange);

  // Ready beeps
  tone(BEEPER_PIN, 1300, 100);
  delay(100);
  tone(BEEPER_PIN, 1500, 100);
  delay(100);
}

void ultrasonicLoop()
{
  distance = ultrasonic.read();
  // Serial.println(distance);
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
  EEPROM.begin(); // Init storage

  stepRange = EEPROM.read(1); // Read saved scan range

  // Setup outputs
  pinMode(FAN_MOTOR, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP); // button needs internal pullup
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  myServo.attach(SERVO_PIN);
  ir.enableIRIn();

  // Run individual setup functions
  stepperSetup();
  encoderSetup(); // <- contains initial configuration stuff
}

void loop()
{

  if (!running) {
    irLoop();
    delay(100); // Breathe
    return;
  }

  stepperLoop(); // Move motor
  ultrasonicLoop(); // Take reading
  servoLoop();

  // Process data to determine if threshold is broken
  if (distance < TRIGGER_DISTANCE_CM)
  {
    bool beep = false;

    // Setup beep time before the motor and leds turnon
    unsigned long beepTime = millis() + 1000L;

    for (;;) {
      // Check ir signal and stop if requested
      irLoop();
      if (!running) break;

      // Break alert if the threshold is no longer broken
      if (ultrasonic.read() > TRIGGER_DISTANCE_CM) break;

      // For the first beepTime ms, only beep
      if (millis() <= beepTime)
      {
        if (beep) tone(BEEPER_PIN, 2000, 100);
        delay(100);
        beep = !beep;
      }
      else // After beepTime ms, toggle red/blue leds and turn on motor
      {
        digitalWrite(FAN_MOTOR, HIGH); // <- will be called on each blink
        digitalWrite(BLUE_LED, beep);
        digitalWrite(RED_LED, !beep);
        beep = !beep;

        delay(100);
      }
    }

    // Turn off fan and leds when threshold is not broken.
    digitalWrite(FAN_MOTOR, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, LOW);
  }
  delay(5);
}