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

bool calibrated = false;
bool basicMode = false;

//cpr 2400 is equivalent to a unit of distance which is  proportional to the diameter of the wheels
const float CPR = 2400.0f;
float vel = 15000.0f;
float velHigh = vel * 1.5;
float velMedium = vel;
float velLow = vel * 0.7;

float forwardRev = 1.0f;
float forwardBoundRev = 0.4f;
float inverseRev = 1.5f;
float turnRev = 1.6f;
float reverseRev = 1.1f;
float scanRev = 3.0f;
float tornadoRev = 7.0f;
float snakeRev = 0.2f;

unsigned long forwardDuration = 0;
unsigned long forwardBoundDuration = 0;
unsigned long inverseDuration = 0;
unsigned long turnDuration = 0;
unsigned long reverseDuration = 0;
unsigned long scanDuration = 0;
unsigned long tornadoDuration = 0;
unsigned long snakeDuration = 0;

unsigned long infraredThreshold = 500;
long distThreshold = 70;
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
                    GS_BATTLE_CHARGE,
                    GS_BATTLE_EDGE,
                    GS_BATTLE_SCAN,
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
  setVelocityMax(30000.0f);
  setCurrentMax(30.0f);

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
      if (move_state != MS_IDLE) {
        brake();
      }
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
      bool edgeTrig = rightEdgeTrig || leftEdgeTrig;
      game_state = (edgeTrig) ? GS_BATTLE_EDGE : game_state;

//      if (!edgeTrig && game_state != GS_BATTLE_EDGE) {
//        if (distance() < distThreshold && game_state != GS_BATTLE_CHARGE) {
//          Serial.println(F("TARGET SPOTTED!"));
//          game_state = GS_BATTLE_CHARGE;
//        }
//      }
      
      if (game_state == GS_BATTLE_MAIN) {
        if (move_state != MS_FORWARD) {
          Serial.println(F("FORWARD"));
          forward(velMedium);
          if (!basicMode) {
            setTimer(random(forwardDuration - forwardBoundDuration, forwardDuration + forwardBoundDuration));
          }
        }
        else if (eventTrigger) {
          long randNumber = random(100);
          if (randNumber < 10) { //turn
            game_state = GS_BATTLE_EDGE;
          }
          else if (randNumber < 30) { //scan
            game_state = GS_BATTLE_SCAN;
          }
          else if (randNumber < 45) { //tornado
            game_state = GS_BATTLE_TORNADO;
          }
//          else if (randNumber < 90) { //snake
//            game_state = GS_BATTLE_SNAKE;
//          }
          else { //charge
            game_state = GS_BATTLE_CHARGE;
          }
        }
      }
      else if (game_state == GS_BATTLE_CHARGE) {
        if (move_state != MS_FORWARD) {
          Serial.println(F("CHARGE"));
          forward(velMedium);
        }
      }
      else if (game_state == GS_BATTLE_EDGE) {
          if (move_state == MS_FORWARD) {
            Serial.println(F("EDGE"));
            hardBrake();
          }
          
          if ((rightEdgeTrig && leftEdgeTrig)) {
            if (random(2)) {
              right(velLow);
            }
            else {
              left(velLow);
            }
          }
          else if (rightEdgeTrig) {
            left(velLow);
          }
          else if (leftEdgeTrig) {
            right(velLow);
          }
          delay(turnDuration);
          brake();
          game_state = GS_BATTLE_MAIN;
      }
      else if (game_state == GS_BATTLE_SCAN) {
        if (move_state != MS_RIGHT && move_state != MS_LEFT) {
          Serial.println(F("SCAN"));
          if (random(2)) {
            right(velLow);
          }
          else {
            left(velLow);
          }
          setTimer(scanDuration);
        }
        setVelocity(0,0);
        delay(100);
        bool distTrig = distance() < distThreshold;
        if (distTrig) {
          Serial.println(F("TARGET SPOTTED!"));
          game_state = GS_BATTLE_CHARGE;
        }
        else {
          if (eventTrigger) {
            brake();
            game_state = GS_BATTLE_MAIN;
          }
          else if (move_state == MS_RIGHT) {
            right(velLow);
            delay(100);
          }
          else if (move_state == MS_LEFT) {
            left(velLow);
            delay(100);
          }
        }
      }
      else if (game_state == GS_BATTLE_TORNADO) {
        if (move_state != MS_RIGHT && move_state != MS_LEFT) {
          Serial.println(F("TORNADO"));
          if (random(1)) {
            right(velHigh);
          }
          else {
            left(velHigh);
          }
          setTimer(tornadoDuration);
        }
        
        else if (eventTrigger) {
          hardBrake();
          game_state = GS_BATTLE_MAIN;
        }
      }
      else if (game_state == GS_BATTLE_SNAKE) {
        if (move_state != MS_RIGHT && move_state != MS_LEFT) {
          Serial.println(F("SNAKE"));
          if (random(1)) {
            move_state = MS_RIGHT;
            setVelocity(velMedium, velLow);
          }
          else {
            move_state = MS_LEFT;
            setVelocity(velLow, velMedium);
          }
          setTimer(snakeDuration);
        }
        else if (eventTrigger) {
          if (move_state == MS_RIGHT) {
            move_state = MS_LEFT;
            setVelocity(velLow, velMedium);
          }
          else {
            move_state = MS_RIGHT;
            setVelocity(velMedium, velLow);
          }
          setTimer(snakeDuration);
        }
        
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
      else if (inputCommand == "set_vel" || inputCommand == "set_forward" || inputCommand == "set_forward_bound" || inputCommand == "set_inverse" || inputCommand == "set_turn" || inputCommand == "set_reverse" || inputCommand == "set_scan" || inputCommand == "set_tornado" || inputCommand == "set_snake") {
        if (inputCommand == "set_vel") {
          vel = inputParam.toFloat();
        }
        else if (inputCommand == "set_forward") {
          forwardRev = inputParam.toFloat();
        }
        else if (inputCommand == "set_forward_bound") {
          forwardBoundRev = inputParam.toFloat();
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
        else if (inputCommand == "set_scan") {
          scanRev = inputParam.toFloat();
        }
        else if (inputCommand == "set_tornado") {
          tornadoRev = inputParam.toFloat();
        }
        else if (inputCommand == "set_snake") {
          snakeRev = inputParam.toFloat();
        }
        updateTimingParams();
        Serial.println(F("Parameter updated!"));
      }
      else if (inputCommand == "set_basicMode") {
        if (inputParam.toInt()) {
          Serial.println(F("Basic Mode ON"));
          basicMode = true;
        }
        else {
          Serial.println(F("Basic Mode OFF"));
          basicMode = false;
        }
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
        Serial.println(F("Max velocity updated!"));
      }
      else if (inputCommand == "pre_calibrate") {
        Serial.println(F("DOES NOT WORK - Setting pre-calibration status!"));
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
        Serial << "Velocity: " << vel << ", IR Threshold: " << infraredThreshold << ", Distance Threshold: " << distThreshold << '\n';

        Serial << F("Forward revolutions: ") << forwardRev << F(", Forward Duration: ") << forwardDuration << '\n';
        Serial << F("Forward bound revolutions: ") << forwardBoundRev << F(", Forward bound Duration: ") << forwardBoundDuration << '\n';
        Serial << F("Inverse revolutions: ") << inverseRev << F(", Inverse Duration: ") << inverseDuration << '\n';
        Serial << F("Turn revolutions: ") << turnRev << F(", Turn Duration: ") << turnDuration << '\n';
        Serial << F("Reverse revolutions: ") << reverseRev << F(", Reverse Duration: ") << reverseDuration << '\n';
        Serial << F("Scan revolutions: ") << scanRev << F(", Scan Duration: ") << scanDuration << '\n';
        Serial << F("Tornado revolutions: ") << tornadoRev << F(", Tornado Duration: ") << tornadoDuration << '\n';
        Serial << F("Snake revolutions: ") << snakeRev << F(", Snake Duration: ") << snakeDuration << '\n';
        
        
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
        calibration();
      }
      else if (inputCommand == "full_calibrate") {
        fullCalibration();
      }
      else if (inputCommand == "save") {
        Serial.println(F("DOES NOT WORK - Saved config!"));
        odrive_serial << "ss\n";
      }
      else if (inputCommand == "erase") {
        Serial.println(F("DOES NOT WORK - Erased config!"));
        odrive_serial << "se\n";
      }
      else if (inputCommand == "reboot") {
        Serial.println(F("DOES NOT WORK - Rebooting odrive!"));
        odrive_serial << "sr\n";
      }
      else if (inputCommand == "remote_control" && calibrated) {
        game_state = GS_REMOTE_CONTROL;
        Serial.println(F("Mode: Remote Control"));
      }
      else if (inputCommand == "start" && calibrated) {
        
        for (int i = 5; i >= 0; i--) {
          Serial << F("Countdown: ") << i << '\n';
          delay(1000);
        }
        game_state = GS_BATTLE_MAIN;
        Serial.println(F("Mode: Battle"));
      }
      else if (inputCommand == "quick_start" && calibrated) {
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
        Serial.println("Error: " + inputCommand + " is an invalid command or odrive is uncalibrated...");
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
  Serial.println(F("Send the string...'max_vel','max_current' [float]"));
  Serial.println(F("Send the string...'pre_calibrate' [1,0]"));
  Serial.println(F("Send the string...'calibrate','full_calibrate','save','erase','reboot'\n"));
  
  Serial.println(F("Parameters Command List..."));
  Serial.println(F("Send the string...'set_vel','set_forward','set_forward_bound','set_inverse','set_turn','set_reverse','set_scan','set_tornado','set_snake' [float]"));
  Serial.println(F("Send the string...'set_infraredThreshold','set_distThreshold' [integer]"));
  Serial.println(F("Send the string...'set_resistance','set_inductance' [float]"));
  Serial.println(F("Send the string...'invert'\n"));
  
  Serial.println(F("Movement Command List..."));
  Serial.println(F("Send the string...'forward','left','right','reverse' [float]"));
  Serial.println(F("Send the string...'forward','forward','left','right','reverse'\n"));
  
  Serial.println(F("State Command List..."));
  Serial.println(F("Send the string...'remote_control','start','quick_start','stop'\n"));
}


void updateTimingParams() {
  float velHigh = vel * 1.5;
  float velMedium = vel;
  float velLow = vel * 0.7;
  
  forwardDuration = revolutionsToMicroseconds(CPR, velMedium, forwardRev);
  forwardBoundDuration = revolutionsToMicroseconds(CPR, velMedium, forwardBoundRev);
  inverseDuration = revolutionsToMicroseconds(CPR, velMedium, inverseRev);
  turnDuration = revolutionsToMicroseconds(CPR, velLow, turnRev);
  reverseDuration = revolutionsToMicroseconds(CPR, velMedium, reverseRev);
  scanDuration = revolutionsToMicroseconds(CPR, velLow, scanRev);
  tornadoDuration = revolutionsToMicroseconds(CPR, velHigh, tornadoRev);
  snakeDuration = revolutionsToMicroseconds(CPR, velMedium, snakeRev);
}

unsigned long revolutionsToMicroseconds(float cpr, float vel_arg, float rev) {
  return ((cpr * rev) / vel_arg) * 1000.0f;
}

float odriveAxisReadFloat(int axis, String property) {
  odrive_serial << "r axis" << axis << property << '\n';
  return odrive.readFloat();
}

void odriveAxisWriteFloat(int axis, String property, float val) {
  odrive_serial << "w axis" << axis << property << ' ' << val << '\n';
}

void calibration() {
  for (int motornum = 0; motornum < 2; motornum++) {
    char c = motornum + '0';
    int requested_state;
    
    requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, true);
    
    requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
    odrive.run_state(motornum, requested_state, false); // don't wait

    calibrated = true;
  }
  
}

void fullCalibration() {
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

    calibrated = true;
  }
}

void setVelocityMax(float vel_arg) {
  for (int axis = 0; axis < 2; ++axis) {
    odriveAxisWriteFloat(axis, ".controller.config.vel_limit", vel_arg);
//    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << vel_arg << '\n';
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

void moveAction(String action, float vel_arg) {
  if (inputCommand == "forward") {
    forward(vel_arg);
    Serial.println("Forward OK");
  } 
  else if (inputCommand == "left") {
    left(vel_arg);
    Serial.println("Left OK");
  }
  else if (inputCommand == "right") {
    right(vel_arg);
    Serial.println("Right OK");
  }
  else if (inputCommand == "reverse") {
    reverse(vel_arg);
    Serial.println("Reverse OK");
  }
}

void setVelocity(float velAxis0, float velAxis1) {
  odrive.SetVelocity(0, -velAxis0); //left
  odrive.SetVelocity(1, velAxis1);
}

void forward(float vel_arg) {
  move_state = MS_FORWARD;
  setVelocity(vel_arg, vel_arg);
}

void right(float vel_arg) {
  move_state = MS_RIGHT;
  setVelocity(vel_arg, - vel_arg);
}

void left(float vel_arg) {
  move_state = MS_LEFT;
  setVelocity(-vel_arg, vel_arg);
}

void reverse(float vel_arg) {
  move_state = MS_REVERSE;
  setVelocity(-vel_arg, -vel_arg);
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
  if (move_state == MS_FORWARD) reverse(velHigh);
  else if (move_state == MS_LEFT) right(velHigh);
  else if (move_state == MS_RIGHT) left(velHigh);
  else if (move_state == MS_REVERSE) forward(velHigh);
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
