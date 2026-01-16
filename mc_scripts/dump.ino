#include <AccelStepper.h>

#define IR_SENSOR A0         // IR Sensor connected to Analog Pin A0
#define DISTANCE_THRESHOLD 5 // Threshold distance in cm

// Define motor control pins
#define DIR_MOTOR1 2
#define STEP_MOTOR1 6
#define DIR_MOTOR2 4
#define STEP_MOTOR2 5

// Define mecanum drive control pins
#define LEFT_MOTOR 6
#define RIGHT_MOTOR 7
#define SIDEWAYS_LEFT 8
#define SIDEWAYS_RIGHT 9

// Create stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, STEP_MOTOR1, DIR_MOTOR1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_MOTOR2, DIR_MOTOR2);

bool mecanum_mode = false; // Mecanum mode flag

void setup()
{
    Serial.begin(9600);

    // Initialize stepper motors
    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(500);
    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(500);

    pinMode(LEFT_MOTOR, OUTPUT);
    pinMode(RIGHT_MOTOR, OUTPUT);
    pinMode(SIDEWAYS_LEFT, OUTPUT);
    pinMode(SIDEWAYS_RIGHT, OUTPUT);
}

void loop()
{
    if (Serial.available())
    {
        char command = Serial.read();
        if (command == 'M')
        {
            mecanum_mode = true;
            startMecanumRoutine();
        }
        else if (command == 'D')
        {
            mecanum_mode = false;
        }
    }
}

// Function to read IR sensor and return distance in cm
int getDistance()
{
    int sensorValue = analogRead(IR_SENSOR);
    return map(sensorValue, 0, 1023, 0, 30); // Adjust if needed
}

void moveBackwardUntilClose()
{
    while (getDistance() > DISTANCE_THRESHOLD)
    {
        Serial.println("Too far, moving back...");
        digitalWrite(LEFT_MOTOR, HIGH);
        digitalWrite(RIGHT_MOTOR, HIGH);
        delay(100);
    }
    digitalWrite(LEFT_MOTOR, LOW);
    digitalWrite(RIGHT_MOTOR, LOW);
    Serial.println("Position reached.");
}

void moveStepper(AccelStepper &stepper, int steps)
{
    stepper.move(steps);
    while (stepper.distanceToGo() != 0)
    {
        stepper.run();
    }
}

void moveBins()
{
    Serial.println("Moving Stepper 1...");
    moveStepper(stepper1, 2000);
    Serial.println("Stepper 1 done.");
}

void moveSideways(int time_ms)
{
    Serial.println("Moving sideways...");
    digitalWrite(SIDEWAYS_LEFT, HIGH);
    digitalWrite(SIDEWAYS_RIGHT, HIGH);
    delay(time_ms);
    digitalWrite(SIDEWAYS_LEFT, LOW);
    digitalWrite(SIDEWAYS_RIGHT, LOW);
}

void moveStepper2()
{
    Serial.println("Moving Stepper 2...");
    moveStepper(stepper2, 1000);
    Serial.println("Stepper 2 done.");
}

void startMecanumRoutine()
{
    Serial.println("Starting mecanum routine...");
    moveBackwardUntilClose();
    moveBins();

    for (int i = 0; i < 2; i++)
    {
        moveSideways(1000);
        moveStepper2();
    }

    Serial.println("Mecanum routine complete.");
}
