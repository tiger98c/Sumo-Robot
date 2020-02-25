/*
 * Sumo remote bluetooth control
 */
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <QTRSensors.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(6, 7); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

// ODrive object
ODriveArduino odrive(odrive_serial);


QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];


const int echoPin = 8;
const int trigPin = 9;

float vel = 7000;
int turn = 750;
int threshold = 500;
int rev = 500;
bool invert = false;

String inputString = "";
String numString = "";
String state = "IDLE";

bool msgReceived = false;

void setup() {
  // put your setup code here, to run once:

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3}, SensorCount);

  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  Serial.begin(9600); // Default communication rate of the Bluetooth module
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 15.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
  Serial.println("Send the character 'c, c0' or 'c1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the string    'd' to read from the ultrasonic sensor");
  Serial.println("Send the string    'r' to read from the QTR sensor array");
  Serial.println("Send the string    'forward[t_ms]' to go forward");
  Serial.println("Send the string    'left[t_ms]' to go left");
  Serial.println("Send the string    'right[t_ms]' to go right");
  Serial.println("Send the string    'reverse[t_ms]' to go reverse");
  Serial.println("Send the string    'vel[num]'");
  Serial.println("Send the string    'turn[num]'");
  Serial.println("Send the string    'rev[num]'");
  Serial.println("Send the string    'threshold[num]'");
  Serial.println("Send the string    'current[num]'");
  Serial.println("Send the string    'maxvel[num]'");
  Serial.println("Send the string    'invert'");
}

void loop() {
  if (Serial.available()) { // Checks whether data is comming from the serial port
    msgReceived = true;
    char inChar = "";
    while (inChar != '\n') {
      if (Serial.available()) {
        inChar = (char)Serial.read(); //read the input
        if (inChar == '\n') {
          break;
        }
        else if (isDigit(inChar)) {
          numString += inChar;
        }
        else {
          inputString += inChar;        //make a string of the characters coming on serial
        }
      }
      delay(10);
    }
  }

  if (msgReceived) {
    if (numString != "") {
      if (inputString == "forward") {
        forward(numString.toInt());
        Serial.println("Forward OK");
      } 
      else if (inputString == "left") {
        left(numString.toInt());
        Serial.println("Left OK");
      }
      else if (inputString == "right") {
        right(numString.toInt());
        Serial.println("Right OK");
      }
      else if (inputString == "reverse") {
        reverse(numString.toInt());
        Serial.println("Reverse OK");
      }
      else if (inputString == "vel") {
        vel = numString.toInt();
      }
      else if (inputString == "threshold") {
        threshold = numString.toInt();
      }
      else if (inputString == "turn") {
        turn = numString.toInt();
      }
      else if (inputString == "rev") {
        rev = numString.toInt();
      }
      else if (inputString == "current") {
        for (int axis = 0; axis < 2; ++axis) {
          odrive_serial << "w axis" << axis << ".motor.config.current_lim " << numString.toInt() << '\n';
          // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
        }
      }
      else if (inputString == "maxvel") {
        for (int axis = 0; axis < 2; ++axis) {
          odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << numString.toInt() << '\n';
          // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
        }
      }
      else if (inputString == "c") {
        int motornum = numString.toInt();
        char c = motornum + '0';
        int requested_state;
        
        requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        odrive.run_state(motornum, requested_state, true);
        
        requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        odrive.run_state(motornum, requested_state, true);
        
        requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        odrive.run_state(motornum, requested_state, false); // don't wait
      }  
      else {
        Serial.println(inputString + " is an invalid command...");
      }
    }
    else { //operations that do not need a numerical argument
      if (inputString == "d") {
        Serial.print("Distance: ");
        Serial.print(distance());
        Serial.println("cm");
      }
      else if (inputString == "invert") {
         invert = !invert;
      }
      else if (inputString == "r") {
        Serial.print("Reflectance: ");
        qtr.read(sensorValues);
        for (uint8_t i = 0; i < SensorCount; i++)
          {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
          }
        Serial.println("");
      }
      else if (inputString == "b") {
        odrive_serial << "r vbus_voltage\n";
        Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
      }
      else if (inputString == "start") {
        delay(5000);
        state = "BATTLE";
      }
      else if (inputString == "qs") {
        state = "BATTLE";
      }
      else if (inputString == "stop") {
        state = "IDLE";
      }
      else if (inputString == "c") {
        for (int motornum = 0; motornum < 2; motornum++) {
//          int motornum = numString.toInt();
          char c = motornum + '0';
          int requested_state;
          
          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
          odrive.run_state(motornum, requested_state, true);
          
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
          odrive.run_state(motornum, requested_state, true);
          
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
          odrive.run_state(motornum, requested_state, false); // don't wait
        }
      }  
      else {
        Serial.println(inputString + " is an invalid command...");
      }
    }
  }
  
  if (state == "IDLE") {
    brake();
    delay(10);
  }
  else if (state == "BATTLE") {
    qtr.read(sensorValues);
    if ((sensorValues[0] > threshold && invert) || (sensorValues[0] < threshold && !invert)) {
      reverse(rev);
      left(turn);
    }
    else if ((sensorValues[3] > threshold && invert) || (sensorValues[3] < threshold && !invert)) {
      reverse(rev);
      right(turn);
    }
    else {
      odrive.SetVelocity(0, -vel); //left
      odrive.SetVelocity(1, vel);
 
      delay(10);
    }
  }
  
  inputString = "";
  numString = "";
  msgReceived = false; 
}

void forward(unsigned long ms) {
  odrive.SetVelocity(0, -vel); //left
  odrive.SetVelocity(1, vel);
 
  delay(ms);
  brake();
}

void right(unsigned long ms) {
  odrive.SetVelocity(0, -vel);
  odrive.SetVelocity(1, -vel);
  delay(ms);
  brake();
}

void left(unsigned long ms) {
  odrive.SetVelocity(0, vel);
  odrive.SetVelocity(1, vel);
  delay(ms);
  brake();
}

void reverse(unsigned long ms) {
  odrive.SetVelocity(0, vel);
  odrive.SetVelocity(1, -vel);
  delay(ms);
  brake();
}

void brake(){
  odrive.SetVelocity(0, 0.0f);
  odrive.SetVelocity(1, 0.0f);
}

long pingDuration() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH);
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long distance() {
  return microsecondsToCentimeters(pingDuration());
}

