#include <Wire.h> // For I2C (optional)

#define IR_SENSOR A0         // IR Sensor connected to Analog Pin A0
#define DISTANCE_THRESHOLD 5 // Threshold distance in cm

// Define stepper motor control pins
#define DIR_MOTOR1 2
#define STEP_MOTOR1 3
#define DIR_MOTOR2 4
#define STEP_MOTOR2 5

// Define mecanum drive control pins
#define LEFT_MOTOR 6
#define RIGHT_MOTOR 7
#define SIDEWAYS_LEFT 8
#define SIDEWAYS_RIGHT 9

bool mecanum_mode = false;

void setup()
{
    Serial.begin(9600);

    pinMode(STEP_MOTOR1, OUTPUT);
    pinMode(DIR_MOTOR1, OUTPUT);
    pinMode(STEP_MOTOR2, OUTPUT);
    pinMode(DIR_MOTOR2, OUTPUT);

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
    int distance = map(sensorValue, 0, 1023, 0, 30);
    return distance;
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

// Function to move stepper motor manually
void moveStepper(int dirPin, int stepPin, int steps)
{
    digitalWrite(dirPin, HIGH);

    for (int i = 0; i < steps; i++)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
    }
}

void moveBins()
{
    Serial.println("Moving Stepper 1...");
    moveStepper(DIR_MOTOR1, STEP_MOTOR1, 2000);
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
    moveStepper(DIR_MOTOR2, STEP_MOTOR2, 1000);
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
