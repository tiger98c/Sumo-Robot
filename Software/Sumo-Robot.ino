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

//cpr 2400 is equivalent to a unit of distance which is  proportional to the diameter of the wheels
const float CPR = 2400.0f;
float vel = 15000.0f; 

float inverseRev = 0.3f;
float turnRev = 4.6f;
float reverseRev = 3.1f;
unsigned long inverseDuration = 0;
unsigned long turnDuration = 0;
unsigned long reverseDuration = 0;
unsigned long tornadoDuration = 2000;

unsigned long infraredThreshold = 500;
long distThreshold = 20;
bool invert = false;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;
bool counterActive = false;

enum move_state_t {MS_IDLE, MS_FORWARD, MS_LEFT, MS_RIGHT, MS_REVERSE};
move_state_t move_state = MS_IDLE;

const unsigned long TIMEOUT = 200;

enum game_state_t { GS_IDLE,
                    GS_REMOTE_CONTROL,
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
  
  updateTimingParams();

  help();
}


void loop() {
  
  bool eventTrigger = false;
  
  unsigned long currentMillis = millis();
  if ((currentMillis >= previousMillis) && counterActive) {
    counterActive = false;
    eventTrigger = true;
  }

  switch(game_state) {
    case GS_IDLE:
      
      break;
    case GS_REMOTE_CONTROL:
      if (move_state != MS_IDLE && eventTrigger) {
        brake();
      }
      break;
    default:
      qtr.read(sensorValues);
      bool rightEdgeTrig = (sensorValues[0] > infraredThreshold && invert) || (sensorValues[0] < infraredThreshold && !invert);
      bool leftEdgeTrig = (sensorValues[1] > infraredThreshold && invert) || (sensorValues[1] < infraredThreshold && !invert);
      game_state = (rightEdgeTrig || leftEdgeTrig) ? GS_BATTLE_EDGE : game_state;
      
      bool distTrig = distThreshold < distance();
      
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
      break;
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
        moveAction(inputCommand, vel);
        delay(inputParam.toFloat());
        hardBrake();
      }
      else if (inputCommand == "set_vel" || inputCommand == "set_inverse" || inputCommand == "set_turn" || inputCommand == "set_reverse") {
        if (inputCommand == "set_vel") {
          vel = inputParam.toFloat();
        }
        else if (inputCommand == "set_inverse") {
          inverseRev = inputParam.toFloat();
        }
        else if (inputCommand == "set_turn") {
          turnRev = inputParam.toFloat();
        }
        else if (inputCommand == "set_reverse") {
          reverseRev = inputParam.toFloat();
        }
        updateTimingParams();
        Serial.println(F("Parameter updated!"));
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
      else {
        Serial.println("Error: " + inputCommand + " is an invalid command...");
      }
    }
    else { //operations that do not need a numerical argument
      if (inputCommand == "info") {
        Serial << "Velocity: " << vel << ", IR Threshold: " << infraredThreshold << ", Distance Threshold: " << distThreshold << '\n';
        
        Serial << "Inverse revolutions: " << inverseRev << ", Inverse Duration: " << inverseDuration << '\n';
        Serial << "Turn revolutions: " << turnRev << ", Turn Duration: " << turnDuration << '\n';
        Serial << "Reverse revolutions: " << reverseRev <<", Reverse Duration: " << reverseDuration << '\n';
        
        
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
          Serial << ", Phase inductance: " << odriveAxisReadFloat(axis, ".motor.config.phase_inductance") << "\n\n";
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
      else if (inputCommand == "remote_control") {
        game_state = GS_REMOTE_CONTROL;
        Serial.println(F("Mode: Remote Control"));
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
      else if (inputCommand == "forward" || inputCommand == "left" || inputCommand == "right" || inputCommand == "reverse") {
        if (game_state == GS_REMOTE_CONTROL) {
          moveAction(inputCommand, vel);
          setTimer(TIMEOUT);
        }
        else {
          Serial.println(F("Error: Robot is not in remote control mode..."));
        }
      }
      else if (inputCommand == "invert") {
         invert = !invert;
         Serial.println(F("Invert updated!"));
      }
      else {
        Serial.println("Error: " + inputCommand + " is an invalid command...");
      }
    }
    
    inputCommand = "";
    inputParam = "";
    stringComplete = false;
  }
}


void help() {
  Serial.println(F("Help Command List..."));
  Serial.println(F("Send the string...'info','help'\n"));
  
  Serial.println(F("Odrive Command List..."));
  Serial.println(F("Send the string...'calibrate','save','erase','reboot'"));
  Serial.println(F("Send the string...'pos_gain','vel_gain','vel_integrator_gain' [float]"));
  Serial.println(F("Send the string...'set_resistance','set_inductance' [float]"));
  Serial.println(F("Send the string...'max_vel','max_current' [float]\n"));
  
  Serial.println(F("Parameters Command List..."));
  Serial.println(F("Send the string...'set_vel','set_inverse','set_turn','set_reverse' [float]"));
  Serial.println(F("Send the string...'set_infraredThreshold','set_distThreshold' [integer]"));
  Serial.println(F("Send the string...'set_resistance','set_inductance' [float]"));
  Serial.println(F("Send the string...'max_vel','max_current' [float]"));
  Serial.println(F("Send the string...'invert'\n"));
  
  Serial.println(F("Movement Command List..."));
  Serial.println(F("Send the string...'forward','left','right','reverse' [float]"));
  Serial.println(F("Send the string...'forward','forward','left','right','reverse'\n"));
  
  Serial.println(F("State Command List..."));
  Serial.println(F("Send the string...'remote_control','start','quick_start','stop'\n"));
}


void updateTimingParams() {
  inverseDuration = revolutionsToMicroseconds(CPR, vel, inverseRev);
  turnDuration = revolutionsToMicroseconds(CPR, vel, turnRev);
  reverseDuration = revolutionsToMicroseconds(CPR, vel, reverseRev);
}

unsigned long revolutionsToMicroseconds(float cpr, float vel, float rev) {
  return ((cpr * rev) / vel) * 1000.0f;
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

void setTimer(unsigned long duration) {
  previousMillis = millis() + duration;
  counterActive = true;
}

void moveAction(String action, float vel) {
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

void hardBrake() {
  inverseMove();
  delay(inverseDuration);
  brake();

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
