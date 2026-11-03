#include <AccelStepper.h>
#include <Preferences.h>

#define STEP_PIN 26
#define DIR_PIN  27

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Preferences preferences;

// Calibration
const long STEPS_FOR_5CM = 4968;
const float STEPS_PER_CM = STEPS_FOR_5CM / 5.0;

// Physical position limits
const float MIN_POSITION_CM = 15.0;
const float MAX_POSITION_CM = 60.0;

// Maximum internal step position
const long MAX_POSITION_STEPS =
    lround((MAX_POSITION_CM - MIN_POSITION_CM) * STEPS_PER_CM);

// Tracks whether the motor was moving previously
bool motorWasMoving = false;

// Prevent repeated saving of the same completed position
long lastSavedPositionSteps = -1;

// Indicates whether a valid saved position was found
bool restoredFromMemory = false;


long cmToSteps(float positionCm)
{
  float distanceFromMinimum = positionCm - MIN_POSITION_CM;

  return lround(distanceFromMinimum * STEPS_PER_CM);
}


float stepsToCm(long stepPosition)
{
  return MIN_POSITION_CM +
         ((float)stepPosition / STEPS_PER_CM);
}


bool positionWithinBounds(long stepPosition)
{
  return stepPosition >= 0 &&
         stepPosition <= MAX_POSITION_STEPS;
}


void saveCurrentPosition()
{
  long currentSteps = motor.currentPosition();

  if (!positionWithinBounds(currentSteps)) {
    Serial.println("ERROR: Current position is outside the valid range.");
    Serial.println("Position was not saved.");
    return;
  }

  preferences.begin("motor", false);

  preferences.putLong("position", currentSteps);
  preferences.putBool("valid", true);

  preferences.end();

  lastSavedPositionSteps = currentSteps;

  Serial.print("Position saved: ");
  Serial.print(stepsToCm(currentSteps), 2);
  Serial.print(" cm, ");
  Serial.print(currentSteps);
  Serial.println(" steps.");
}


bool restoreSavedPosition()
{
  preferences.begin("motor", true);

  bool validPosition =
      preferences.getBool("valid", false);

  long savedSteps =
      preferences.getLong("position", 0);

  preferences.end();

  if (!validPosition) {
    return false;
  }

  if (!positionWithinBounds(savedSteps)) {
    Serial.println("WARNING: Stored position is out of bounds.");
    Serial.println("Stored position will not be used.");

    return false;
  }

  motor.setCurrentPosition(savedSteps);
  lastSavedPositionSteps = savedSteps;

  return true;
}


void forgetSavedPosition()
{
  if (motor.isRunning()) {
    Serial.println("ERROR: Stop the motor before clearing its position.");
    return;
  }

  preferences.begin("motor", false);
  preferences.clear();
  preferences.end();

  restoredFromMemory = false;
  lastSavedPositionSteps = -1;

  Serial.println();
  Serial.println("Saved motor position cleared.");
  Serial.println("Physically move the motor to 15 cm, then enter 'zero'.");
}


void goToPosition(float requestedPositionCm)
{
  if (requestedPositionCm < MIN_POSITION_CM ||
      requestedPositionCm > MAX_POSITION_CM) {

    Serial.println();
    Serial.println("ERROR: Requested position is out of bounds.");

    Serial.print("Requested position: ");
    Serial.print(requestedPositionCm, 2);
    Serial.println(" cm");

    Serial.print("Allowed range: ");
    Serial.print(MIN_POSITION_CM, 2);
    Serial.print(" cm to ");
    Serial.print(MAX_POSITION_CM, 2);
    Serial.println(" cm");

    return;
  }

  long targetSteps = cmToSteps(requestedPositionCm);

  Serial.println();

  Serial.print("Current calculated position: ");
  Serial.print(stepsToCm(motor.currentPosition()), 2);
  Serial.println(" cm");

  Serial.print("Moving to: ");
  Serial.print(requestedPositionCm, 2);
  Serial.println(" cm");

  Serial.print("Target step position: ");
  Serial.println(targetSteps);

  motor.moveTo(targetSteps);

  motorWasMoving = true;
}

void setKnownPosition(float knownPositionCm)
{
  if (motor.isRunning()) {
    Serial.println();
    Serial.println("ERROR: Cannot set position while the motor is moving.");
    Serial.println("Send 'stop' and wait until the motor has stopped.");
    return;
  }

  if (knownPositionCm < MIN_POSITION_CM ||
      knownPositionCm > MAX_POSITION_CM) {

    Serial.println();
    Serial.println("ERROR: Known position is out of bounds.");

    Serial.print("Entered position: ");
    Serial.print(knownPositionCm, 2);
    Serial.println(" cm");

    Serial.print("Allowed range: ");
    Serial.print(MIN_POSITION_CM, 2);
    Serial.print(" cm to ");
    Serial.print(MAX_POSITION_CM, 2);
    Serial.println(" cm");

    return;
  }

  long knownPositionSteps = cmToSteps(knownPositionCm);

  /*
    Sets both the current and target positions to this value.
    This does not physically move the motor.
  */
  motor.setCurrentPosition(knownPositionSteps);

  restoredFromMemory = false;
  motorWasMoving = false;

  Serial.println();
  Serial.println("Current position reference updated.");

  Serial.print("Physical position defined as: ");
  Serial.print(knownPositionCm, 2);
  Serial.println(" cm");

  Serial.print("Internal step position: ");
  Serial.println(knownPositionSteps);

  saveCurrentPosition();
}

void stopMotor()
{
  if (motor.distanceToGo() == 0) {
    Serial.println("Motor is already stationary.");
    return;
  }

  motor.stop();

  Serial.println();
  Serial.println("STOP command received.");
  Serial.println("Motor is decelerating to a stop.");
}


void showPosition()
{
  long currentSteps = motor.currentPosition();
  long targetSteps = motor.targetPosition();

  Serial.println();

  Serial.print("Current calculated position: ");
  Serial.print(stepsToCm(currentSteps), 2);
  Serial.println(" cm");

  Serial.print("Current internal position: ");
  Serial.print(currentSteps);
  Serial.println(" steps");

  Serial.print("Target position: ");
  Serial.print(stepsToCm(targetSteps), 2);
  Serial.println(" cm");

  Serial.print("Target internal position: ");
  Serial.print(targetSteps);
  Serial.println(" steps");

  if (motor.distanceToGo() == 0) {
    Serial.println("Motor status: stationary");
  }
  else {
    Serial.println("Motor status: moving");
  }

  if (restoredFromMemory) {
    Serial.println("Startup reference: restored from memory");
  }
  else {
    Serial.println("Startup reference: manually zeroed/default");
  }
}


void zeroMotorAtMinimum()
{
  if (motor.isRunning()) {
    Serial.println();
    Serial.println("ERROR: Cannot zero while the motor is moving.");
    Serial.println("Send 'stop' and wait until it stops.");
    return;
  }

  /*
    Only use this while the mechanism is physically
    located at the 15 cm mark.
  */
  motor.setCurrentPosition(0);
  restoredFromMemory = false;

  Serial.println();
  Serial.println("Current step position set to zero.");
  Serial.println("Physical position is now defined as 15 cm.");

  saveCurrentPosition();
}


bool isValidPositionNumber(String text)
{
  if (text.length() == 0) {
    return false;
  }

  bool decimalFound = false;

  for (unsigned int i = 0; i < text.length(); i++) {
    char character = text.charAt(i);

    if (character == '.' && !decimalFound) {
      decimalFound = true;
    }
    else if (!isDigit(character)) {
      return false;
    }
  }

  return true;
}


void processCommand(String command)
{
  command.trim();
  command.toLowerCase();

  if (command == "stop") {
    stopMotor();
  }
  else if (command == "position") {
    showPosition();
  }
  else if (command == "zero") {
    zeroMotorAtMinimum();
  }
  else if (command == "save") {
    if (motor.isRunning()) {
      Serial.println("ERROR: Wait for the motor to stop before saving.");
    }
    else {
      saveCurrentPosition();
    }
  }
  else if (command == "forget") {
    forgetSavedPosition();
  }
  else if (command.startsWith("set ")) {
    String positionText = command.substring(4);
    positionText.trim();

    if (!isValidPositionNumber(positionText)) {
      Serial.println("ERROR: Invalid position.");
      Serial.println("Example: set 32");
      return;
    }

    float knownPositionCm = positionText.toFloat();

    setKnownPosition(knownPositionCm);
  }
  else if (command == "help") {
    printCommands();
  }
  else if (command.startsWith("go ")) {
    String positionText = command.substring(3);
    positionText.trim();

    if (!isValidPositionNumber(positionText)) {
      Serial.println("ERROR: Invalid position.");
      Serial.println("Example: go 40");
      return;
    }

    float requestedPositionCm = positionText.toFloat();

    goToPosition(requestedPositionCm);
  }
  else if (command.length() > 0) {
    Serial.print("ERROR: Unknown command: ");
    Serial.println(command);
    Serial.println("Enter 'help' to display the commands.");
  }
}


void printCommands()
{
  Serial.println();
  Serial.println("Available commands:");
  Serial.println("  go X      : move to X cm");
  Serial.println("  go 15     : move to the lower position");
  Serial.println("  go 60     : move to the upper position");
  Serial.println("  position  : show current and target positions");
  Serial.println("  stop      : decelerate and stop");
  Serial.println("  save      : save the current position manually");
  Serial.println("  zero      : define current physical position as 15 cm");
  Serial.println("  set X     : define the current physical position as X cm");
  Serial.println("  forget    : erase the saved position");
  Serial.println("  help      : show this command list");
  Serial.println();
}


void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(50);

  motor.setMinPulseWidth(3);
  motor.setMaxSpeed(800);
  motor.setAcceleration(400);

  Serial.println();
  Serial.println("ESP32 stepper controller starting.");

  restoredFromMemory = restoreSavedPosition();

  if (restoredFromMemory) {
    Serial.println("Saved motor position restored.");

    Serial.print("Restored calculated position: ");
    Serial.print(stepsToCm(motor.currentPosition()), 2);
    Serial.println(" cm");

    Serial.print("Restored step position: ");
    Serial.println(motor.currentPosition());

    Serial.println();
    Serial.println("WARNING:");
    Serial.println("This position has not been physically verified.");
    Serial.println("Confirm that the mechanism has not been moved");
    Serial.println("while the ESP32 was switched off.");
  }
  else {
    /*
      This is the first startup or the saved position
      has been deliberately cleared.

      The mechanism must be physically at 15 cm.
    */
    motor.setCurrentPosition(0);

    Serial.println("No valid saved position was found.");
    Serial.println("Current position is being assumed to be 15 cm.");
    Serial.println("Verify the physical position before moving.");

    saveCurrentPosition();
  }

  Serial.println();
  Serial.println("Motor will remain stationary until a command is received.");

  printCommands();
}


void loop()
{
  motor.run();

  /*
    Detect when a movement has finished.

    The position is saved only once after the motor stops,
    rather than writing continuously while it moves.
  */
  if (motorWasMoving && motor.distanceToGo() == 0) {
    motorWasMoving = false;

    Serial.println();
    Serial.println("Movement complete.");

    Serial.print("Final calculated position: ");
    Serial.print(stepsToCm(motor.currentPosition()), 2);
    Serial.println(" cm");

    if (motor.currentPosition() != lastSavedPositionSteps) {
      saveCurrentPosition();
    }
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
}