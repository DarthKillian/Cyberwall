#include <Arduino.h>
#include <elapsedMillis.h>

// --- Constants and Pin Definitions ---
#define TILT_UP_BTN 7              // Pin for tilting the wall up
#define TILT_DOWN_BTN 8            // Pin for tilting the wall down
#define LOWER_LIMIT A2             // Pin for lower limit switch
#define UPPER_LIMIT A1             // Pin for upper limit switch
#define NUMBER_OF_ACTUATORS 2      // Number of actuators in the system
#define SAMPLES_PER_MEASUREMENT 50 // Number of steps to measure timing
#define FALSE_PULSE_DELAY 8        // Debounce delay for hall sensors (ms)
#define BASE_SPEED 100             // Base PWM speed for actuators
#define OFFSET_MULTIPLIER 5        // Multiplier for speed offset in synchronization

// --- Pin Assignments for Actuators ---
const int FPWM[NUMBER_OF_ACTUATORS] = {9, 5};  // Forward PWM pins
const int RPWM[NUMBER_OF_ACTUATORS] = {10, 6}; // Reverse PWM pins
const int HALL[NUMBER_OF_ACTUATORS] = {2, 3};  // Hall sensor pins

// --- Data Structure ---
struct ActuatorData
{
  int speed;
  volatile long steps;
  volatile long prevSteps;
};

// --- Global Variables ---
elapsedMillis timeElapsed;                                          // Timer for elapsed time measurements
unsigned long lastLogTime = 0;                                      // Last time actuator status was logged
ActuatorData actuators[NUMBER_OF_ACTUATORS];                        // Array for actuator data
int direction = 0;                                                  // Movement direction: 1 (forward), -1 (reverse), 0 (stop)
volatile unsigned long lastDebounceTime[NUMBER_OF_ACTUATORS] = {0}; // Debounce Timers

// --- Function Prototypes ---
void driveActuators();
void moveToLimit(int speed);
bool haveStepsChanged();
void logActuatorStatus();
void counter_0();
void counter_1();

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < NUMBER_OF_ACTUATORS; i++)
  {
    actuators[i] = {0, 0, 0};

    pinMode(FPWM[i], OUTPUT);
    pinMode(RPWM[i], OUTPUT);
    pinMode(HALL[i], OUTPUT);
    lastDebounceTime[i] = 0;
  }

  pinMode(TILT_UP_BTN, INPUT_PULLUP);
  pinMode(TILT_DOWN_BTN, INPUT_PULLUP);
  pinMode(LOWER_LIMIT, INPUT_PULLUP);
  pinMode(UPPER_LIMIT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL[0]), counter_0, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL[1]), counter_1, RISING);
}

void loop()
{
}

// --- Drive Actuators ---
/** Controls actuator movement and synchronizes their speeds. */
void driveActuators()
{
  long totalSteps = 0;
  for (int i = 0; i < NUMBER_OF_ACTUATORS; i++)
  {
    totalSteps += actuators[i].steps;
  }
  long avgSteps = totalSteps / NUMBER_OF_ACTUATORS;

  for (int i = 0; i < NUMBER_OF_ACTUATORS; i++)
  {
    int offset = (avgSteps - actuators[i].steps) * OFFSET_MULTIPLIER * direction;
    actuators[i].speed = min(abs(BASE_SPEED + offset), 255);
    switch (direction)
    {
    case 1: // Forward
      analogWrite(FPWM[i], actuators[i].speed);
      analogWrite(RPWM[i], 0);
      break;
    case -1: // Reverse
      analogWrite(FPWM[i], 0);
      analogWrite(RPWM[i], actuators[i].speed);
      break;
    default: // Stop
      analogWrite(FPWM[i], 0);
      analogWrite(RPWM[i], 0);
      break;
    }
  }
  logActuatorStatus(); // Log status periodically
}

// --- Log Actuator Status ---
/** Logs actuator speed and position every second. */
void logActuatorStatus()
{
  if (millis() - lastLogTime >= 1000)
  {
    lastLogTime = millis();
    Serial.print("Actuator 1 speed: ");
    Serial.print(actuators[0].speed);
    Serial.print(" | Pos: ");
    Serial.print(actuators[0].steps);
    Serial.print(" | Actuator 2 speed: ");
    Serial.print(actuators[1].speed);
    Serial.print(" | Pos: ");
    Serial.println(actuators[1].steps);
  }
}

// --- Move to Limit ---
/** Moves actuators until they hit a limit (steps stop changing). */
void moveToLimit(int speed) {
  Serial.println("Moving to limit...");
  for (int i = 0; i < NUMBER_OF_ACTUATORS; i++) {
    actuators[i].steps = 0;
    actuators[i].prevSteps = 0;
  }
  do {
    for (int i = 0; i < NUMBER_OF_ACTUATORS; i++) {
      actuators[i].prevSteps = actuators[i].steps;
    }
    timeElapsed = 0;
    while (timeElapsed < 200) {
      for (int i = 0; i < NUMBER_OF_ACTUATORS; i++) {
        analogWrite(FPWM[i], direction == 1 ? speed : 0);
        analogWrite(RPWM[i], direction == -1 ? speed : 0);
      }
    }
  } while (haveStepsChanged());
}

// --- Check Step Changes ---
/** Returns true if any actuator's step count has changed. */
bool haveStepsChanged() {
  for (int i = 0; i < NUMBER_OF_ACTUATORS; i++) {
    if (actuators[i].prevSteps != actuators[i].steps) {
      return true;
    }
  }
  return false;
}

// --- Interrupt Handlers ---
void counter_0() {
  if (millis() - lastDebounceTime[0] > FALSE_PULSE_DELAY) {
    lastDebounceTime[0] = millis();
    actuators[0].steps += direction;
  }
}

void counter_1() {
  if (millis() - lastDebounceTime[1] > FALSE_PULSE_DELAY) {
    lastDebounceTime[1] = millis();
    actuators[1].steps += direction;
  }
}