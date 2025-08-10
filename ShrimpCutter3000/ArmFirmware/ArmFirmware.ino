// Send two integer numbers indicating angle values of the servo. The first number is
// for the big servo and the second number for the small servo, e.g. 40,50.
//
// According to the library, the angle range is from 0-180. To get finer control over the angle
// one could use servo.writeMicroseconds() instead of servo.write(). See
// https://github.com/arduino-libraries/Servo/blob/master/docs/api.md

#include <Servo.h> // By Arduino, v1.2.1

// Define servo objects
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

// Define servo pins
const int servo0Pin = 8;
const int servo1Pin = 9;
const int servo2Pin = 10;
const int servo3Pin = 11;
const int servo4Pin = 12;
const int servo5Pin = 13;

// Define the maximum length of the input string
const int BUFFER_SIZE = 64;

// Define the delimiter character
const char DELIMITER = ',';

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use when parsing

// variables to hold the parsed data
int actuator = 0;
int value = 0;

// Flag to indicate whether a complete set of values has been received
bool newData = false;

// Create a buffer to store the incoming serial data
char buffer[BUFFER_SIZE];

// flags for attaching the servos
bool servo0attached = false;
bool servo1attached = false;
bool servo2attached = false;
bool servo3attached = false;
bool servo4attached = false;
bool servo5attached = false;

// variables to measure the activation time for the solenoids
unsigned long activationTime46 = 0;
unsigned long activationTime48 = 0;

unsigned long lastActivationTime = 0;

void setup()
{
    
    // set servo pins to output and LOW
    // pinMode(servo0Pin, OUTPUT); // sets the digital pin 30 as output
    // digitalWrite(servo0Pin, LOW);
    // pinMode(servo1Pin, OUTPUT); // sets the digital pin 30 as output
    // digitalWrite(servo1Pin, LOW);
    // pinMode(servo2Pin, OUTPUT); // sets the digital pin 30 as output
    // digitalWrite(servo2Pin, LOW);
    // pinMode(servo3Pin, OUTPUT); // sets the digital pin 30 as output
    // digitalWrite(servo3Pin, LOW);
    // pinMode(servo4Pin, OUTPUT); // sets the digital pin 30 as output
    // digitalWrite(servo4Pin, LOW);
    // pinMode(servo5Pin, OUTPUT); // sets the digital pin 30 as output
    // digitalWrite(servo5Pin, LOW);
    // set solenoid pins to output and LOW
    pinMode(42, OUTPUT); // sets the digital pin 30 as output
    digitalWrite(42, LOW);
    pinMode(44, OUTPUT); // sets the digital pin 32 as output
    digitalWrite(44, LOW);
    pinMode(46, OUTPUT); // sets the digital pin 32 as output
    digitalWrite(44, LOW);
    pinMode(48, OUTPUT); // sets the digital pin 32 as output
    digitalWrite(44, LOW);
    delay(500);
    Serial.begin(9600);
}

void loop()
{
    // read serial for new Data
    recvWithStartEndMarkers();
    // if we have new data, parse it
    if (newData == true)
    {
        strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        newData = false;
    }
    // de-energize solenoids after 1.5 seconds
    if (millis() - activationTime46 > 1500)
    {
        digitalWrite(46, LOW);
    }

    if (millis() - activationTime48 > 1500)
    {
        digitalWrite(48, LOW);
    }

    if (millis() - lastActivationTime > 1500)
    {
        servo0.detach();
        servo0attached = false;
        servo1.detach();
        servo1attached = false;
        servo2.detach();
        servo2attached = false;
        servo3.detach();
        servo3attached = false;
        servo4.detach();
        servo4attached = false;
        servo5.detach();
        servo5attached = false;
    }
    // Small delay to debounce input readings
    delay(50);
}

void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false)
    {
        rc = Serial.read();

        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

void parseData()
{ // split the data into its parts

    char *strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ","); // get the first part - the string
    actuator = atoi(strtokIndx);         // copy it to messageFromPC

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    value = atoi(strtokIndx);       // convert this part to an integer
    // Use the values to control the servos
    switch (actuator)
    {
    case 0:
        if (value > 499)
        {
            if (servo0attached == false)
            {
                servo0.attach(servo0Pin);
                servo0attached = true;
            }
            servo0.writeMicroseconds(value);
            lastActivationTime = millis();
        }
        break;

    case 1:
        if (value > 499)
        {
            if (servo1attached == false)
            {
                servo1.attach(servo1Pin);
                servo1attached = true;
            }
            servo1.writeMicroseconds(value);
            lastActivationTime = millis();
        }
        break;

    case 2:
        if (value > 499)
        {
            if (servo2attached == false)
            {
                servo2.attach(servo2Pin);
                servo2attached = true;
            }
            servo2.writeMicroseconds(value);
            lastActivationTime = millis();
        }
        break;

    case 3:
        if (value > 499)
        {
            if (servo3attached == false)
            {
                servo3.attach(servo3Pin);
                servo3attached = true;
            }
            servo3.writeMicroseconds(value);
            lastActivationTime = millis();
        }
        break;

    case 4:
        if (value > 499)
        {
            if (servo4attached == false)
            {
                servo4.attach(servo4Pin);
                servo4attached = true;
            }
            servo4.writeMicroseconds(value);
            lastActivationTime = millis();
        }
        break;

    case 5:
        if (value > 499)
        {
            if (servo5attached == false)
            {
                servo5.attach(servo5Pin);
                servo5attached = true;
            }
            servo5.writeMicroseconds(value);
            lastActivationTime = millis();
        }
        break;

    case 42:
        if (value == 1)
        {
           
            digitalWrite(42, HIGH);
        }
        else
        {
            digitalWrite(42, LOW);
        }
        break;

    case 44:
        if (value == 1)
        {
   
            digitalWrite(44, HIGH);
        }
        else
        {
            digitalWrite(44, LOW);
        }

        break;

    case 46:
        if (value == 1)
        {
            activationTime46 = millis();
            digitalWrite(46, HIGH);
        }
        else
        {
            digitalWrite(46, LOW);
        }

        break;

    case 48:
        if (value == 1)
        {
            activationTime48 = millis();
            digitalWrite(48, HIGH);
        }
        else
        {
            digitalWrite(48, LOW);
        }

        break;
    // detach servos at app exit and set solenoids to LOW
    case 77:
        servo0.detach();
        servo1.detach();
        servo2.detach();
        servo3.detach();
        servo4.detach();
        servo5.detach();
        digitalWrite(30, LOW);
        digitalWrite(32, LOW);
    }
}
