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

float vel = 15000.0f;
unsigned long inverseDuration = 50;
unsigned long turnDuration = 750;
unsigned long revDuration = 500;
unsigned long infraredThreshold = 500;
long distThreshold = 20;
bool invert = false;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long interval = 0;

enum move_state_t {MS_IDLE, MS_FORWARD, MS_LEFT, MS_RIGHT, MS_REVERSE};
move_state_t move_state = MS_IDLE;

enum game_state_t {GS_IDLE, 
                   GS_BATTLE_MAIN, 
                   GS_BATTLE_EDGE,
                   GS_BATTLE_SCAN,
                   GS_BATTLE_TARGET,
                   GS_BATTLE_TORNADO,
                   GS_BATTLE_SNAKE};
game_state_t game_state = GS_IDLE;


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

  help();
}

void loop() {

  if (game_state == GS_IDLE) {
    if (move_state != MS_IDLE) {
      move_state = MS_IDLE;
      brake();
    }
  }
  else {
    qtr.read(sensorValues);
    bool rightEdgeTrig = (sensorValues[0] > infraredThreshold && invert) || (sensorValues[0] < infraredThreshold && !invert);
    bool leftEdgeTrig = (sensorValues[1] > infraredThreshold && invert) || (sensorValues[1] < infraredThreshold && !invert);
    game_state = (rightEdgeTrig || leftEdgeTrig) ? GS_BATTLE_EDGE : game_state;
    
    bool distTrig = distThreshold < distance();
    
    unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis >= interval) && (interval != 0)) {
  //    previousMillis = currentMillis;
  //    interval = 0;
    }
    
    if (game_state == GS_BATTLE_MAIN) {
      
    }
    else if (game_state == GS_BATTLE_EDGE) {
      if (rightEdgeTrig && leftEdgeTrig) {
        if (random(1)) {
          right(vel);
        }
        else {
          left(vel);
        }
        interval = turnDuration;
      }
    }
    else if (game_state == GS_BATTLE_SCAN) {
      
    }
    else if (game_state == GS_BATTLE_TARGET) {
      
    }
    else if (game_state == GS_BATTLE_TORNADO) {
      
    }
    else if (game_state == GS_BATTLE_SNAKE) {
      
    }
  }

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
      else if (inputCommand == "set_inverse") {
        inverseDuration = inputParam.toInt();
        Serial.println(F("Inverse duration updated!"));
      }
      else if (inputCommand == "set_turn") {
        turnDuration = inputParam.toInt();
        Serial.println(F("Turn duration updated!"));
      }
      else if (inputCommand == "set_rev") {
        revDuration = inputParam.toInt();
        Serial.println(F("Reverse duration updated!"));
      }
      else if (inputCommand == "set_infraredThreshold") {
        infraredThreshold = inputParam.toInt();
        Serial.println(F("IR threshold updated!"));
      }
      else if (inputCommand == "set_distThreshold") {
        distThreshold = inputParam.toInt();
        Serial.println(F("Distance threshold updated!"));
      }
      else if (inputCommand == "pos_gain") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".controller.config.pos_gain", inputParam.toFloat());
        }
        Serial.println(F("Position gain updated!"));
      }
      else if (inputCommand == "vel_gain") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".controller.config.vel_gain", inputParam.toFloat());
        }
        Serial.println(F("Velocity gain updated!"));
      }
      else if (inputCommand == "vel_integrator_gain") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".controller.config.vel_integrator_gain", inputParam.toFloat());
        }
        Serial.println(F("Velocity integrator gain updated!"));
      }
      else if (inputCommand == "set_resistance") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".motor.config.phase_resistance", inputParam.toFloat());
        }
        Serial.println(F("Phase resistance updated!"));
      }
      else if (inputCommand == "set_inductance") {
        for (int axis = 0; axis < 2; axis++) {
          odriveAxisWriteFloat(axis, ".motor.config.phase_inductance", inputParam.toFloat());
        }
        Serial.println(F("Phase inductance updated!"));
      }
      else if (inputCommand == "max_current") {
        setCurrentMax(inputParam.toFloat());
        Serial.println(F("Max current updated!"));
      }
      else if (inputCommand == "max_vel") {
        setVelocityMax(inputParam.toFloat());
        
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
      else {
        Serial.println("Error: " + inputCommand + " is an invalid command...");
      }
    }
    else { //operations that do not need a numerical argument
      if (inputCommand == "info") {
        Serial << "Velocity: " << vel << ", IR Threshold: " << infraredThreshold << ", Distance Threshold: " << distThreshold
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
          Serial << "Axis " << axis << "\nVelocity limit: " << odriveAxisReadFloat(axis, ".controller.config.vel_limit");
          Serial << ", Current limit: " << odriveAxisReadFloat(axis, ".motor.config.current_lim") << '\n';

          Serial << "Pos gain: " << odriveAxisReadFloat(axis, ".controller.config.pos_gain");
          Serial << ", Vel gain: " << odriveAxisReadFloat(axis,".controller.config.vel_gain");
          Serial << ", Vel integrator gain: " << odriveAxisReadFloat(axis, ".controller.config.vel_integrator_gain") << "\n";
          
          Serial << "Phase resistance: " << odriveAxisReadFloat(axis, ".motor.config.phase_resistance");
          Serial << ", Phase inductance: " << odriveAxisReadFloat(axis, ".motor.config.phase_inductance") << '\n';
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
        game_state = GS_BATTLE_MAIN;
        Serial.println(F("Mode: Battle"));
      }
      else if (inputCommand == "quick_start") {
        game_state = GS_BATTLE_MAIN;
        Serial.println(F("Mode: Battle"));
      }
      else if (inputCommand == "stop") {
        game_state = GS_IDLE;
        Serial.println(F("Mode: Idle"));
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

//          odrive_serial << "r axis" << axis << ".controller.config.vel_limit\n";
//          odrive_serial << "r axis" << axis << ".motor.config.current_lim\n";
//          odrive_serial << "r axis" << axis <<".controller.config.pos_gain\n";
//          odrive_serial << "r axis" << axis <<".controller.config.vel_gain\n";
//          odrive_serial << "r axis" << axis <<".controller.config.vel_integrator_gain\n";
//          odrive_serial << "r axis" << axis <<".motor.config.phase_resistance\n";
//          odrive_serial << "r axis" << axis <<".motor.config.phase_inductance\n";

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
  odrive_serial << "w axis" << axis << property << ' ' << val << '\n';
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
  //properly initialise motor properties to be even
  //getting uneven reverse/forward
  //this seems to work!!!!
  delay(ms);
  inverseMove();
  delay(inverseDuration);
  brake();
}

void setVelocity(float velAxis0, float velAxis1) {
  odrive.SetVelocity(0, -velAxis0); //left
  odrive.SetVelocity(1, velAxis1);
}

void forward(float vel) {
  move_state = MS_FORWARD;
  setVelocity(vel, vel);
}

void right(float vel) {
  move_state = MS_RIGHT;
  setVelocity(vel, - vel);
}

void left(float vel) {
  move_state = MS_LEFT;
  setVelocity(-vel, vel);
}

void reverse(float vel) {
  move_state = MS_REVERSE;
  setVelocity(-vel, -vel);
}

void brake() {
  move_state = MS_IDLE;
  setVelocity(0.0f, 0.0f);
}

void inverseMove() {
  if (move_state == MS_FORWARD) reverse(vel);
  else if (move_state == MS_LEFT) right(vel);
  else if (move_state == MS_RIGHT) left(vel);
  else if (move_state == MS_REVERSE) forward(vel);
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
