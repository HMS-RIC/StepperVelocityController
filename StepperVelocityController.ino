#include "src/XNucleoDualStepperDriver/XNucleoDualStepperDriver.h"
#include "src/XNucleoDualStepperDriver/dSPINConstants.h"
#include <SPI.h>
#include "PID.h"

// Motor setup
const float Vsupply = 12;   // DC supply voltage
const float Vmotor = 4.5;   // motor's nominal voltage (in V)
const byte Ithresh = OCD_TH_750mA; // over-current threshold

const float Kaccl = 1.2; // fraction of full voltage for acceleration
const float Krun  = 1.0; // fraction of full voltage for const. vel.
const float Khold = 0.5; // fraction of full voltage for holding

// Velocity/accleration profile:
// From stop, motor will first jump to minSpeed, then accelerate at accelRate up to (at most) maxSpeed.
// Deceleration rate is also set to be accelRate.
const int minSpeed = 160; // in steps/s;
const int maxSpeed = 1000; // in steps/s
const int accelRate = 5000; // in steps/s^2
const int fullSpeed = 100; // in steps/s; use microsteps below this speed
const float goToLimitSpeed = 500.0; // in steps/s


// Conversion from steps to physical units (cm, degrees, pixels, etc...):
// ======================================================================
// How many full steps in single motor revolution?
// (property of the stepper motor; typically 200)
const int FULL_STEPS_PER_MOTOR_REV = 200;
// Microstep mode
const int stepMode = STEP_FS_64; // 64 microsteps per step
const int MICROSTEPS_PER_MOTOR_REV = 64 * FULL_STEPS_PER_MOTOR_REV;
// How many phyical units of translation in one complete motor revolution?
const float UNITS_PER_MOTOR_REV = 360;
// We can compute our conversion factor:
const float UNITS_PER_MICROSTEP = UNITS_PER_MOTOR_REV / MICROSTEPS_PER_MOTOR_REV;



// PID / Targetting
bool PIDMode = false;
float currPos = 0;
long currSteps = 0;
float targetPos = 0;
float error = 0;
float newVelocity = 0;
float errorThresh = 0.1; // don't move if abs(error) is below this value

SPid pidState;


// ========  SPI/Motor Settings  ==============
const int resetPin = 4;
const int SCKPin = 13;
const int CSPin = A2;
XNucleoStepper motor(0, CSPin, resetPin);

void DEBUG(String message) {
  // Serial.println(message.c_str()); // comment this out when not debugging !!!!!!!
}


void setup()
{
  Serial.begin(115200);

  // Set P, I, and D gain values
  pidState.propGain = 1;     // proportional gain
  pidState.integratGain = 0;  // integral gain
  pidState.derGain = 0;    // derivative gain
  // Limit integrator values to avoid windup (due to reaching motor's top velocity)
  pidState.integratMax = 1000;  // Maximum and minimum
  pidState.integratMin = -1000;  // allowable integrator state
  // Set internal states to 0
  pidState.derState = 0;     // Last position input
  pidState.integratState = 0;  // Integrator state

  // Start by setting up the pins and the SPI peripheral.
  //  The library doesn't do this for you!
  configSPI();
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH);
  configMotor(&motor);

  // reset alarms on both motors (to turn off (red) alarm LED
  // on XNucleo board)
  XNucleoStepper motor2(1, CSPin, resetPin);
  motor2.getAlarmStatusString();
  motor.getAlarmStatusString();
}

unsigned long prevTime = micros();
bool motorBusy = false;
bool prevMotorBusy = false;
unsigned long prevDebugTime = micros();
void loop()
{
  // debugging code to get loop time:
  //    (as of 1/2017 loop takes ~32 microseconds)
  // DEBUG(String(micros()-prevTime));
  // delay(100);
  // prevTime = micros();

  readUSB();

}

void readUSB() {
  // Read from USB, if available
  static String usbMessage = ""; // initialize usbMessage to empty string,
                                 // happens once at start of program
  while (Serial.available() > 0) {
    // keep read single chars if available
    char inByte = Serial.read();
    if ((inByte == '\n') || (inByte == ';')) {
      // the new-line character ('\n') or ';' indicate a complete message
      // so interprete the message and then clear buffer
      interpretCommand(usbMessage);
      usbMessage = ""; // clear message buffer
      break; // exit while loop after interpreting command (even if more chars remain)
    } else {
      // append character to message buffer
      usbMessage = usbMessage + inByte;
    }
  }
}

void interpretCommand(String message) {
  message.trim(); // remove leading and trailing white space
  int len = message.length();
  if (len==0) {
    Serial.print("#"); // "#" means error
    Serial.print(message); // added TO
    return;
  }
  char command = message[0]; // the command is the first char of a message
  String parameters = message.substring(1);
  parameters.trim();

  String intString = "";
  while ((parameters.length() > 0) && (isDigit(parameters[0]))) {
    intString += parameters[0];
    parameters.remove(0,1);
  }
  long arg1 = intString.toInt();

  DEBUG(String("Command: ")+command);
  DEBUG(String("Argument 1: ")+arg1);

  switch (command) {

    case 'S': // S: set max speed
    case 's':
      motor.setMaxSpeed(arg1);
      Serial.print("Speed set to ");
      Serial.print(arg1);
      Serial.println(" steps/s");
      break;

    case 'A': // A: set accel rate
    case 'a':
      motor.setAcc(arg1);
      motor.setDec(arg1);
      Serial.print("Speed accel rate to ");
      Serial.print(arg1);
      Serial.println(" steps/s^2");
      break;

    case 'P': // P: set P gain
    case 'p':
      pidState.propGain = arg1;
      Serial.print("Set P gain to ");
      Serial.println(arg1);
      break;

    case 'I': // I: set I gain
    case 'i':
      pidState.integratGain = arg1;
      Serial.print("Set I gain to ");
      Serial.println(arg1);
      break;

    case 'D': // D: set D gain
    case 'd':
      pidState.derGain = arg1;
      Serial.print("Set D gain to ");
      Serial.println(arg1);
      break;


    case 'X': // X: soft stop
    case 'x':
      PIDMode = false;
      motor.softStop();
      Serial.println("Stopping");
      break;

    case 'Q': // Q: Hi-Z (free movements)
    case 'q':
      PIDMode = false;
      motor.softHiZ();
      Serial.println("High-Z (free movement)");
      break;

    case 'F': // F: forward
    case 'f':
      PIDMode = false;
      motor.move(FWD, (float)arg1/UNITS_PER_MICROSTEP);
      Serial.print("Moving Forward ");
      Serial.print(arg1);
      Serial.println(" units");
      break;

    case 'B': // B/R: Reverse
    case 'b':
    case 'R':
    case 'r':
      PIDMode = false;
      motor.move(REV, (float)arg1/UNITS_PER_MICROSTEP);
      Serial.print("Moving Backward ");
      Serial.print(arg1);
      Serial.println(" units");
      break;

    case 'G': // G: Go to position
    case 'g':
      if (arg1 >= 0) {
        PIDMode = false;
        Serial.print("Moving to position ");
        Serial.println(arg1);
        motor.goTo((float)arg1/UNITS_PER_MICROSTEP);
        // while (motor.busyCheck()) {
        //   delay(1);
        // }
      } else {
        Serial.println("GoTo: please provide a non-negative position.");
      }
      break;

    case 'Z': // Z: Re-zero at current position
    case 'z':
      // reset position to limit switch
      PIDMode = false;
      Serial.print("Zeroing");
      motor.resetPos();
      currPos = 0;
      targetPos = 0;
      break;

    case 'H': // H: Home — reset zero position at limit switch
    case 'h':
      // reset position to limit switch
      PIDMode = false;
      Serial.print("Homing... ");
      motor.goUntil(RESET_ABSPOS, REV, goToLimitSpeed);
      while (motor.busyCheck()) {
        delay(1);
      }
      motor.releaseSw(RESET_ABSPOS, FWD);
      while (motor.busyCheck()) {
        delay(1);
      }
      motor.resetPos();
      currPos = 0;
      targetPos = 0;
      Serial.println("Done");
      break;

    case 'T': // T: Track position (PID mode)
    case 't':
      PIDMode = true;
      targetPos = arg1;
      currPos = motor.getPos() * UNITS_PER_MICROSTEP;
      error = (targetPos - currPos);
      newVelocity = UpdatePID(&pidState, error, currPos);
      // TODO maybe stop motor if error or newVel is smaller than some threhold, to prevent jittering.
      if (newVelocity >= 0) {
        motor.run(FWD, newVelocity);
      } else {
        motor.run(REV, -newVelocity);
      }
      break;

    case 'E': // E: Track error (PID mode)
    case 'e':
      PIDMode = true;
      error = arg1;
      currPos = motor.getPos() * UNITS_PER_MICROSTEP;
      targetPos = currPos + error;
      newVelocity = UpdatePID(&pidState, error, currPos);
      // TODO maybe stop motor if error or newVel is smaller than some threhold, to prevent jittering.
      if (newVelocity >= 0) {
        motor.run(FWD, newVelocity);
      } else {
        motor.run(REV, -newVelocity);
      }
      break;

    case 'W': // W: Report position ([W]here am I?)
    case 'w':
      currSteps = motor.getPos();
      currPos = currSteps * UNITS_PER_MICROSTEP;
      Serial.print("Current location: ");
      Serial.print(currPos);
      Serial.print("\t\t (");
      Serial.print(currSteps);
      Serial.println(" steps)");
      break;

    case '?': // ?: Report motor status
      Serial.println(motor.getFullStatusString());
      break;

    case '!': // !: Check Flag status
    // TODO check flag status first....
        Serial.print("Motor-1: ");
        Serial.println(motor.getAlarmStatusString());
      break;

    default: // unknown command
      Serial.println("#"); // "#" means error
  } // switch(command)
}

void configSPI() {
  pinMode(resetPin, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCKPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  digitalWrite(resetPin, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
}

void configMotor(XNucleoStepper* motor)
{
  int paramValue;
  //Serial.println("Configuring motor...");

  // Before we do anything, we need to tell each board which SPI
  //  port we're using. Most of the time, there's only the one,
  //  but it's possible for some larger Arduino boards to have more
  //  than one, so don't take it for granted.
  motor->SPIPortConnect(&SPI);

  // Set the Overcurrent Threshold. The OC detect circuit
  //  is quite sensitive; even if the current is only momentarily
  //  exceeded during acceleration or deceleration, the driver
  //  will shutdown. This is a per channel value; it's useful to
  //  consider worst case, which is startup.
  motor->setOCThreshold(Ithresh);

  // KVAL is a modifier that sets the effective voltage applied
  //  to the motor. KVAL/255 * Vsupply = effective motor voltage.
  //  This lets us hammer the motor harder during some phases
  //  than others, and to use a higher voltage to achieve better
  //  torqure performance even if a motor isn't rated for such a
  //  high current.
  float Kval = Vmotor/Vsupply * 255;
  motor->setRunKVAL(round(Krun*Kval));
  motor->setAccKVAL(round(Kaccl*Kval));
  motor->setDecKVAL(round(Kaccl*Kval));
  motor->setHoldKVAL(round(Khold*Kval));

  // When a move command is issued, max speed is the speed the
  //  motor tops out at while completing the move, in steps/s
  motor->setMinSpeed(minSpeed);
  motor->setMaxSpeed(maxSpeed);
  motor->setFullSpeed(maxSpeed);       // microstep below this speed


  // Acceleration and deceleration in steps/s/s. Increasing this
  //  value makes the motor reach its full speed more quickly,
  //  at the cost of possibly causing it to slip and miss steps.
  motor->setAcc(accelRate);
  motor->setDec(accelRate);

  motor->configStepMode(stepMode);    // microsteps per step


  // // Not sure if we need any of these:
  // motor->configSyncPin(BUSY_PIN, 0);// BUSY pin low during operations;
  //                                   //  second paramter ignored.
  // motor->setSlewRate(SR_530V_us);   // Upping the edge speed increases torque.
  // motor->setPWMFreq(PWM_DIV_2, PWM_MUL_2); // 31.25kHz PWM freq
  // motor->setOCShutdown(OC_SD_DISABLE); // don't shutdown on OC
  // motor->setVoltageComp(VS_COMP_DISABLE); // don't compensate for motor V
  // motor->setSwitchMode(SW_USER);    // Switch is not hard stop
  // motor->setOscMode(INT_16MHZ_OSCOUT_16MHZ); // 16MHz internal oscil



}
