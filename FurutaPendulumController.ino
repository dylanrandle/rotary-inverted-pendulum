#include <Encoder.h>
#include <AccelStepper.h>
#include <AutoPID.h>

const int encoderStepsPerRevolution = 2400;
const double encoderDegreesPerStep = 360. / (double)encoderStepsPerRevolution;
const float ANGLE_MIN = 140;
const float ANGLE_MAX = 220;

Encoder encoder(2, 3);

const int microStepPin1 = 8;
const int microStepPin2 = 9;
const int microStepPin3 = 10;
const int motorStepsPerRevolution = 1600;  // 1/8th microstepping
const double motorDegreesPerStep = 360. / (double)motorStepsPerRevolution;
const int maxSpeed = 100 * motorStepsPerRevolution;
const int maxAcceleration = 100 * motorStepsPerRevolution;

AccelStepper stepper(AccelStepper::DRIVER, 4, 5);

double encoderAngle = 0;
double controlAngle = 0;
double targetAngle = 180;

long controlSteps = 0;

double OUTPUT_MIN = -180;
double OUTPUT_MAX = 180;

double KP = 0.1;
double KI = 10;
double KD = 1;

AutoPID pid(&encoderAngle, &targetAngle, &controlAngle, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  Serial.begin(9600);

  pinMode(microStepPin1, OUTPUT);
  pinMode(microStepPin2, OUTPUT);
  pinMode(microStepPin3, OUTPUT);

  // 1/8th microstep
  digitalWrite(microStepPin1, HIGH);
  digitalWrite(microStepPin2, HIGH);
  digitalWrite(microStepPin3, LOW);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAcceleration);

  pid.setBangBang(10);
  pid.setTimeStep(1);
}

void readEncoder() {
  int pos = encoder.read() % encoderStepsPerRevolution;
  double angle = (double)pos * encoderDegreesPerStep;
  if (angle < 0) {
    angle += 360;
  }
  encoderAngle = angle;
}

unsigned long lastLogTimestamp = millis();
int logInterval = 100;

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

void runController() {
  readEncoder();
  pid.run();
  controlSteps = (long)round(controlAngle / motorDegreesPerStep);
  stepper.move(controlSteps);

  if (encoderAngle <= ANGLE_MAX && encoderAngle >= ANGLE_MIN) {
    stepper.run();
  } else {
    pid.stop();
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
  // runMotorTest();
}
