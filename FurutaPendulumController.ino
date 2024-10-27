#include <Encoder.h>
#include <AccelStepper.h>
#include <AutoPID.h>

// encoder
Encoder encoder(2, 3);
const int encoderStepsPerRevolution = 2400;
const double encoderDegreesPerStep = 360. / (double)encoderStepsPerRevolution;

// stepper
AccelStepper stepper(AccelStepper::DRIVER, 4, 5);
const int motorStepsPerRevolution = 400;
const double motorDegreesPerStep = 360. / (double)motorStepsPerRevolution;
const int maxSpeed = 100 * motorStepsPerRevolution;
const int maxAcceleration = 100 * motorStepsPerRevolution;

// PID
double encoderAngle = 0;
double controlAngle = 0;
double targetAngle = 180;
double minControlAngle = -90;
double maxControlAngle = 90;
double minActiveAngle = 140;
double maxActiveAngle = 220;
double kp = 0.1;
double ki = 10;
double kd = 0;
long controlSteps = 0;
long controlTimestep = 1;
AutoPID pid(&encoderAngle, &targetAngle, &controlAngle, minControlAngle, maxControlAngle, kp, ki, kd);

// logging
unsigned long lastLogTimestamp = millis();
int logInterval = 10;

void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAcceleration);

  pid.setTimeStep(controlTimestep);
}

void readEncoder() {
  int pos = encoder.read() % encoderStepsPerRevolution;
  double angle = (double)pos * encoderDegreesPerStep;
  if (angle < 0) {
    angle += 360;
  }
  encoderAngle = angle;
}

void log() {
  unsigned long currentTimestamp = millis();

  if (currentTimestamp - lastLogTimestamp > logInterval) {
    Serial.print("EncoderAngle:");
    Serial.print(encoderAngle);
    Serial.print(",");
    Serial.print("ControlAngle:");
    Serial.print(controlAngle);
    Serial.print(",");
    Serial.print("ControlSteps:");
    Serial.print(controlSteps);
    Serial.println();
    lastLogTimestamp = currentTimestamp;
  }
}

void stopPID() {
  pid.stop();
  pid.reset();
  controlSteps = 0;
  controlAngle = 0;
}

void updateControlSteps() {
  controlSteps = (long)floor(controlAngle / motorDegreesPerStep);
}

void runController() {
  readEncoder();

  if (encoderAngle <= maxActiveAngle && encoderAngle >= minActiveAngle) {
    pid.run();
    updateControlSteps();
    stepper.move(controlSteps);
    stepper.run();
  } else {
    stepper.stop();
    stopPID();
  }

  log();
}

bool isClockwise = true;

void runMotorTest() {
  long currentPosition = stepper.currentPosition();
  if (isClockwise) {
    stepper.runToNewPosition(currentPosition + motorStepsPerRevolution);
  } else {
    stepper.runToNewPosition(currentPosition - motorStepsPerRevolution);
  }
  delay(1000);
  isClockwise = !isClockwise;
}

void loop() {
  runController();
}
