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
SoftwareSerial odrive_serial(7,6); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

// ODrive object
ODriveArduino odrive(odrive_serial);

QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

const int echoPin = 8;
const int trigPin = 9;

float vel = 7000.0f;
unsigned long turnDuration = 750;
unsigned long threshold = 500;
unsigned long revDuration = 500;
bool invert = false;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long interval = 0;

String state = "IDLE";


void setup() {
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1}, SensorCount);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  Serial.begin(9600); // Default communication rate of the Bluetooth module
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");
  
//  setVelocityMax(22000.0f);
//  setCurrentMax(15.0f);
  
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
//  motorCalibration();

  help();
}

void loop() {

//  qtr.read(sensorValues);
//  bool rightEdgeTrig = (sensorValues[0] > threshold && invert) || (sensorValues[0] < threshold && !invert);
//  bool leftEdgeTrig = (sensorValues[1] > threshold && invert) || (sensorValues[1] < threshold && !invert);
//  
//  unsigned long currentMillis = millis();
//  if ((currentMillis - previousMillis >= interval) && (interval != 0)) {
////    previousMillis = currentMillis;
////    interval = 0;
//  }
//  
//  if (state == "IDLE") {
//    brake();
//    delay(10);
//  }
//  else if (state == "BATTLE") {
//    qtr.read(sensorValues);
//    if ((sensorValues[0] > threshold && invert) || (sensorValues[0] < threshold && !invert)) {
//      reverse(revDuration);
//      left(turnDuration);
//    }
//    else if ((sensorValues[1] > threshold && invert) || (sensorValues[1] < threshold && !invert)) {
//      reverse(revDuration);
//      right(turnDuration);
//    }
//    else {
//      odrive.SetVelocity(0, -vel); //left
//      odrive.SetVelocity(1, vel);
// 
//      delay(10);
//    }
//  }
}


String inputCommand = "";         // a String to hold incoming data
String inputParam = "";
bool stringComplete = false;  // whether the string is complete

void serialEvent() {
    
  while (Serial.available() && ~stringComplete) {
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    
    if (isDigit(inChar) || inChar == '.') {
      inputParam += inChar;
    }
    else if (inChar == '\n') {
      stringComplete = true;
    }
    else if (isSpace(inChar)) {
      continue;
    }
    else {
      // add it to the inputString:
      inputCommand += inChar;
    }
  }
  
  if (stringComplete) {
    if (inputParam != "") {
      if (inputCommand == "forward" || inputCommand == "left" || inputCommand == "right" || inputCommand == "reverse") {
        moveAction(inputCommand, inputParam.toFloat(), vel);
      }
      else if (inputCommand == "set_vel") {
        vel = inputParam.toFloat();
        Serial.println(F("Velocity updated!"));
      }
      else if (inputCommand == "set_threshold") {
        threshold = inputParam.toInt();
        Serial.println(F("Threshold updated!"));
      }
      else if (inputCommand == "set_turn") {
        turnDuration = inputParam.toInt();
        Serial.println(F("Turn duration updated!"));
      }
      else if (inputCommand == "set_rev") {
        revDuration = inputParam.toInt();
        Serial.println(F("Reverse duration updated!"));
      }
      else if (inputCommand == "pre_calibrate") {
        Serial.println(F("Setting pre-calibration status!"));
        for (int axis = 0; axis < 2; ++axis) {
          if (inputParam == "1"){
            odrive_serial << "w axis" << axis << ".motor.config.pre_calibrated " << true << '\n';
          }
          else if (inputParam == "0"){
            odrive_serial << "w axis" << axis << ".motor.config.pre_calibrated " << false << '\n';
          }
          else {
            Serial.println(F("Error: Invalid boolean, put a \'1\' or \'0\'."));
          }
        }
      }
      else if (inputCommand == "pos_gain") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".controller.config.pos_gain", inputParam.toFloat());
        }
      }
      else if (inputCommand == "vel_gain") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".controller.config.vel_gain", inputParam.toFloat());
        }
      }
      else if (inputCommand == "vel_integrator_gain") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".controller.config.vel_integrator_gain", inputParam.toFloat());
        }
      }
      else if (inputCommand == "max_current") {
        setCurrentMax(inputParam.toFloat());
        Serial.println(F("Max current updated!"));
      }
      else if (inputCommand == "max_vel") {
        setVelocityMax(inputParam.toFloat());
        Serial.println(F("Max velocity updated!"));
      }
      else {
        Serial.println("Error: " + inputCommand + " is an invalid command...");
      }
    }
    else { //operations that do not need a numerical argument
      if (inputCommand == "info") {
        Serial << "Velocity: " << vel << ", Threshold: " << threshold 
               << ", Turn Duration: " << turnDuration << ", Reverse Duration: " << revDuration << '\n';
        
        Serial << "Distance: " << distance() << "cm\n";
        
        Serial.print("Reflectance: ");
        qtr.read(sensorValues);
        for (uint8_t i = 0; i < SensorCount; i++) {
          Serial << sensorValues[i] << '\t';
        }
        Serial << '\n';

        odrive_serial << "r vbus_voltage\n";
        Serial << "Vbus voltage: " << odrive.readFloat() << '\n';

        for (int axis = 0; axis < 2; ++axis) {
//          odrive_serial << "r axis" << axis << ".controller.config.vel_limit\n";
          Serial << "Axis " << axis << "\nVelocity limit: " << odriveAxisReadFloat(axis, ".controller.config.vel_limit");
//          odrive_serial << "r axis" << axis << ".motor.config.current_lim\n";
          Serial << ", Current limit: " << odriveAxisReadFloat(axis, ".motor.config.current_lim") << '\n';

//          odrive_serial << "r axis" << axis <<".controller.config.pos_gain\n";
          Serial << "Pos gain: " << odriveAxisReadFloat(axis, ".controller.config.pos_gain\n");
//          odrive_serial << "r axis" << axis <<".controller.config.vel_gain\n";
          Serial << ", Vel gain: " << odriveAxisReadFloat(axis,".controller.config.vel_gain\n");
//          odrive_serial << "r axis" << axis <<".controller.config.vel_integrator_gain\n";
          Serial << ", Vel integrator gain: " << odriveAxisReadFloat(axis, ".controller.config.vel_integrator_gain\n") << "\n";
          
//          odrive_serial << "r axis" << axis <<".motor.config.phase_resistance\n";
          Serial << "Phase resistance: " << odriveAxisReadFloat(axis, ".motor.config.phase_resistance\n");
//          odrive_serial << "r axis" << axis <<".motor.config.phase_inductance\n";
          Serial << ", Phase inductance: " << odriveAxisReadFloat(axis, ".motor.config.phase_inductance\n") << "\n\n";
        }
      }
      else if (inputCommand == "help") {
        help();
      }
      else if (inputCommand == "calibrate") {
        motorCalibration();
      }
      else if (inputCommand == "save") {
        Serial.println(F("Saved config!"));
        odrive_serial << "ss\n";
      }
      else if (inputCommand == "erase") {
        Serial.println(F("Erased config!"));
        odrive_serial << "se\n";
      }
      else if (inputCommand == "reboot") {
        Serial.println(F("Rebooting odrive!"));
        odrive_serial << "sr\n";
      }
      else if (inputCommand == "invert") {
         invert = !invert;
         Serial.println(F("Invert updated!"));
      }
      else if (inputCommand == "start") {
        delay(5000);
        for (int i = 5; i >= 0; i--) {
          Serial << F("Countdown: ") << i << '\n';
        }
        state = "BATTLE";
      }
      else if (inputCommand == "quick_start") {
        Serial.println(F("Mode: Battle"));
        state = "BATTLE";
      }
      else if (inputCommand == "stop") {
        Serial.println(F("Mode: Idle"));
        state = "IDLE";
      }
      else {
        Serial.println(inputCommand + " is an invalid command...");
      }
    }
    
    inputCommand = "";
    inputParam = "";
    stringComplete = false;
  }
}


//<axis>.controller.config.pos_gain = 20.0 [(counts/s) / counts]
//<axis>.controller.config.vel_gain = 5.0 / 10000.0 [A/(counts/s)]
//<axis>.controller.config.vel_integrator_gain = 10.0 / 10000.0 [A/((counts/s) * s)]
//An upcoming feature will enable automatic tuning. Until then, here is a rough tuning procedure:
//
//Set vel_integrator_gain gain to 0
//Make sure you have a stable system. If it is not, decrease all gains until you have one.
//Increase vel_gain by around 30% per iteration until the motor exhibits some vibration.
//Back down vel_gain to 50% of the vibrating value.
//Increase pos_gain by around 30% per iteration until you see some overshoot.
//Back down pos_gain until you do not have overshoot anymore.
//The integrator can be set to 0.5 * bandwidth * vel_gain, where bandwidth is the overall resulting 
//tracking bandwidth of your system. Say your tuning made it track commands with a settling time of 100ms
//(the time from when the setpoint changes to when the system arrives at the new setpoint); this means
//the bandwidth was 1/(100ms) = 1/(0.1s) = 10hz. In this case you should set the vel_integrator_gain = 0.5 * 10 * vel_gain.

//Parameter reading/writing
//Not all parameters can be accessed via the ASCII protocol but at least all parameters with float and integer type are supported.
//
//Reading:
// r [property]
//property name of the property, as seen in ODrive Tool
//response: text representation of the requested value
//Example: r vbus_voltage => response: 24.087744
//Writing:
// w [property] [value]
//property name of the property, as seen in ODrive Tool
//value text representation of the value to be written
//Example: w axis0.controller.pos_setpoint -123.456
//System commands:
//ss - Save config
//se - Erase config
//sr - Reboot

void help() {
  Serial.println(F("Ready!"));
  Serial.println(F("Send the character 'c, c0' or 'c1' to calibrate respective motor (you must do this before you can command movement)"));
  Serial.println(F("Send the character 'b' to read bus voltage"));
  Serial.println(F("Send the string    'd' to read from the ultrasonic sensor"));
  Serial.println(F("Send the string    'r' to read from the QTR sensor array"));
  Serial.println(F("Send the string    'forward[t_ms]' to go forward"));
  Serial.println(F("Send the string    'left[t_ms]' to go left"));
  Serial.println(F("Send the string    'right[t_ms]' to go right"));
  Serial.println(F("Send the string    'reverse[t_ms]' to go reverse"));
  Serial.println(F("Send the string    'vel[num]'"));
  Serial.println(F("Send the string    'turn[num]'"));
  Serial.println(F("Send the string    'rev[num]'"));
  Serial.println(F("Send the string    'threshold[num]'"));
  Serial.println(F("Send the string    'current[num]'"));
  Serial.println(F("Send the string    'maxvel[num]'"));
  Serial.println(F("Send the string    'invert'"));
}


float odriveAxisReadFloat(int axis, String property) {
  odrive_serial << "r axis" << axis << property << '\n';
  return odrive.readFloat();
}

void odriveAxisWriteFloat(int axis, String property, float val) {
  odrive_serial << "w axis" << axis << property << val << " \n";
}

void motorCalibration() {
  for (int motornum = 0; motornum < 2; motornum++) {
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

void setVelocityMax(float vel) {
  for (int axis = 0; axis < 2; ++axis) {
    odriveAxisWriteFloat(axis, ".controller.config.vel_limit", vel);
//    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << vel << '\n';
  }
}

void setCurrentMax(float current) {
  for (int axis = 0; axis < 2; ++axis) {
    odriveAxisWriteFloat(axis, ".motor.config.current_lim", current);
//    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << current << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 15.0\n"
  }
}

void moveAction(String action, unsigned long ms, float vel) {
  if (inputCommand == "forward") {
    forward(vel);
    Serial.println("Forward OK");
  } 
  else if (inputCommand == "left") {
    left(vel);
    Serial.println("Left OK");
  }
  else if (inputCommand == "right") {
    right(vel);
    Serial.println("Right OK");
  }
  else if (inputCommand == "reverse") {
    reverse(vel);
    Serial.println("Reverse OK");
  }
  delay(ms);
  brake();
}

void forward(float vel) {
  odrive.SetVelocity(0, -vel); //left
  odrive.SetVelocity(1, vel);
}

void right(float vel) {
  odrive.SetVelocity(0, -vel);
  odrive.SetVelocity(1, -vel);
}

void left(float vel) {
  odrive.SetVelocity(0, vel);
  odrive.SetVelocity(1, vel);
}

void reverse(float vel) {
  odrive.SetVelocity(0, vel);
  odrive.SetVelocity(1, -vel);
}

void brake() {
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
