#include <Wire.h> // include wire library so we can communicate with gyro
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "SparkFun_MS5803_I2C.h"

MS5803 sensor(ADDRESS_HIGH);

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

//===========================================================================
// Data Variables
//===========================================================================
// Motor Data================================================================
int motorStop1 = 1481; // Pulse width to stop motor
int motorStop2 = 1481;
int motorStop3 = 1481;
int motorStop4 = 1481;
int pulse1 = 1481; // Pulse width to be sent to motor in PID control
int pulse2 = 1481;
int pulse3 = 1481;
int pulse4 = 1481;
int thrust = 1; // Step thrust to control motor mannually
int thrust_input = 0; //the additional forward thrust given by the sliding bar

int feedMotor1, feedMotor2, feedMotor3, feedMotor4; // final calculated pulses that are being fed to motor

int limitBand = 481; 
int maxPulse1 = motorStop1 + limitBand; // set upper limit for pulse to motor1
int minPulse1 = motorStop1 - limitBand; // set lower limit for pulse to motor1
int maxPulse2 = motorStop2 + limitBand; 
int minPulse2 = motorStop2 - limitBand;
int maxPulse3 = motorStop3 + limitBand;
int minPulse3 = motorStop3 - limitBand;
int maxPulse4 = motorStop4 + limitBand;
int minPulse4 = motorStop4 - limitBand;


// Control Data==============================================================
double propellerConstant = 0.5; //Accounts for the reverse thrust of boat propeller
double xPropGain = 3.0; // Proportional Gain constant
double yPropGain = 3.0;
double zPropGain = 3.0;
double xPropOutput, yPropOutput, zPropOutput;
int xIntegralGain = 1; //Integral Gain Constant
int yIntegralGain = 1;
int zIntegralGain = 1;
double xIntegralOutput, yIntegralOutput, zIntegralOutput;
int xDerivativeGain = 1; // Derivative Gain Constant
int yDerivativeGain = 1;
int zDerivativeGain = 1;
double xDerivativeOutput, yDerivativeOutput, zDerivativeOutput;
double eulXPrev, eulYPrev, eulZPrev;
double xOutput, yOutput, zOutput;

// Quaternion Data===========================================================
imu::Quaternion Qref; // reference position in quaternion form
double qRef1 = Qref.w(), qRef2 = Qref.x(), qRef3 = Qref.y(), qRef4 = Qref.z(); // Desired/Reference/Home Orientation
double qMeas1, qMeas2, qMeas3, qMeas4; // Actual Orientation from sensor
double qMeasCon1, qMeasCon2, qMeasCon3, qMeasCon4; // Conjugate of measured quaternion to calculate relative error
double qErr1, qErr2, qErr3, qErr4; // Calculated relative error values with respect to inertial frame
double qMeasPrev1 = 1, qMeasPrev2 = 0, qMeasPrev3 = 0, qMeasPrev4 = 0; //Temporary storage variables to filter out sensor error
double qLength; //length of unit quaternion
double eulX, eulY, eulZ; // variables for quaternion to euler conversion
double qRefCon1, qRefCon2, qRefCon3, qRefCon4; //Conjugate of the desired/reference/home orientation
double temp1, temp2, temp3, temp4; //Temporary storage for converting relative error from inertial frame to home/body frame
double qErrHome1, qErrHome2, qErrHome3, qErrHome4; //Relative Error with respect to inertial frame

// update_home_orientation_and_thrust function variable
int Q_recv, T_recv, Depth_recv;
float Q1, Q2, Q3, Q4;
float Depth_con_qRef1 = 0.7;
float Depth_con_qRef2 = 0.7;
float Depth_con_qRef3 = 0.04;
float Depth_con_qRef4 = 0.0;
float Thrust_gain = 0.5;
float Depth_thrust_gain = 10.0;
float Depth_recv_gain = 0.5;
float Desire_depth = 1000.0;
float Current_depth = 1000.0;
//===========================================================================
// Helper Variables
//===========================================================================

int s = 0; // temporary variable to read the serial monitor command
// Quaternion Data===========================================================
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // create sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55,BNO055_ADDRESS_B); // create sensor object
char rx_byte = 0; // trigger variable from serial monitor to change desired orientation
int t = 0;
//===========================================================================
// Main Setup Function
//===========================================================================
void setup() {
  //Inititate motor at stop pulse
  motor1.attach(3);
  motor2.attach(9);
  motor3.attach(5);
  motor4.attach(6);
  motor1.writeMicroseconds(motorStop1);
  motor2.writeMicroseconds(motorStop2);
  motor3.writeMicroseconds(motorStop3);
  motor4.writeMicroseconds(motorStop4);

  Serial.begin(9600); // start the serial monitor for logging

  sensor.reset();
    sensor.begin();
    
//  bno.begin();
  if (bno.begin()) Serial.println("Sensor Initialized"); // initialize the sensor, default mode is NDOF
  Qref = bno.getQuat();// start quaternion and store desired orientation
  Serial.println("CLEARDATA"); //This is for logging data into excel
  Serial.println("LABEL,Current time,x,y,z"); //This is for logging data into excel

}

//===========================================================================
// Main Loop Function
//===========================================================================
void loop()
{
  if (Serial.available())
  {
    char val = Serial.read(); /* read the user input
    x: stop motor
    f: forward
    b: backward
    s: start attitude keeping control
      1: reset home configuration
      0: stop attitude keeping control
*/
    switch (val)
    {
      case 'x':
        pulse1 = motorStop1;
        pulse2 = motorStop2;
        pulse3 = motorStop3;
        pulse4 = motorStop4;
        motor1.writeMicroseconds(pulse1);
        motor2.writeMicroseconds(pulse2);
        motor3.writeMicroseconds(pulse3);
        motor4.writeMicroseconds(pulse4);
        Serial.print("Stop / Pulse - Motor1:"), Serial.print(pulse1), Serial.print("  Motor2:"), Serial.print(pulse2),
        Serial.print("  Motor3:"), Serial.print(pulse3),Serial.print("  Motor4:"), Serial.print(pulse4),Serial.println();
        break;

      case 'f':
        pulse1 = pulse1 + thrust;
        pulse2 = pulse2 + thrust;
        pulse3 = pulse3 + thrust;
        pulse4 = pulse4 + thrust;
        motor1.writeMicroseconds(pulse1);
        motor2.writeMicroseconds(pulse2);
        motor3.writeMicroseconds(pulse3);
        motor4.writeMicroseconds(pulse4);
        Serial.print("Forward / Pulse - Motor1:"), Serial.print(pulse1), Serial.print("  Motor2:"), Serial.print(pulse2),
        Serial.print("  Motor3:"), Serial.print(pulse3),Serial.print("  Motor4:"), Serial.print(pulse4),Serial.println();
        break;

      case 'b':
        pulse1 = pulse1 - thrust;
        pulse2 = pulse2 - thrust;
        pulse3 = pulse3 - thrust;
        pulse4 = pulse4 - thrust;
        motor1.writeMicroseconds(pulse1);
        motor2.writeMicroseconds(pulse2);
        motor3.writeMicroseconds(pulse3);
        motor4.writeMicroseconds(pulse4);
        Serial.print("Backward / Pulse - Motor1: "), Serial.print(pulse1), Serial.print("  Motor2:"), Serial.print(pulse2),
        Serial.print("  Motor3:"), Serial.print(pulse3),Serial.print("  Motor4:"), Serial.print(pulse4),Serial.println();

        break;
        
        
        case 'w':
        for(float i = 0 ; i < 100 ; i=i+0.5 ){
  motor1.write(motorStop1+i); 
  motor2.write(motorStop1+i);
  motor3.write(motorStop1+i); 
  motor4.write(motorStop1+i); 
  delay(10);
  }    
       //  Serial.println("Forward");
         break;

         case 'e':
       for(float i = 0 ; i < 100 ; i=i+0.5 ){
  motor1.write(motorStop1-i); 
  motor2.write(motorStop1-i);
  motor3.write(motorStop1-i); 
  motor4.write(motorStop1-i); 
  delay(10);
  }           
        // Serial.println("Backward");
         break;

      case 's':
        s = 1;
//        Serial.println("Testing commence");
        break;

      default:
        break;

    }


    while (s == 1)
    {
      //Calculate Error===========================================================
      imu::Quaternion Qm = bno.getQuat(); // find the quaternion representing current orientation
      qMeas1 = Qm.w(); // store the current quaternion
      qMeas2 = Qm.x();
      qMeas3 = Qm.y();
      qMeas4 = Qm.z();
      //      filter(); // filter quaternion readings that has a length less than 0.99
     
  //     Current_depth = sensor.getPressure(ADC_4096) - 1000.0;
     
      //trigger(); // check if triggered to change home configuration (press: 1) or to stop attitude keeping (press: 0)
       if (Serial.available() > 0) {
      update_home_orientation_and_thrust_depth();
     // simple_depth_control();
      }
      qError(); // calculate relative orientation error
      qErrorHome(); // Convert relative error from inertial frame to home frame
      euler(); // calculate euler angles from relative quaternion
      //      qMeasPrev1=qMeas1; // store the measured quaternion for later use in filtering
      //      qMeasPrev2=qMeas2;
      //      qMeasPrev3=qMeas3;
      //      qMeasPrev4=qMeas4;
      //==========================================================================
      pidControl(); // caculate PID Control output based on error
      feedMotorPulse(); //Feed the corrected pulse to the motor
     //printData(); // print data to serial monitor

//     Serial.print("Thrust input = ");
//    Serial.print(thrust_input);
//
//    Serial.print("Desire depth = ");
//    Serial.print(Desire_depth);
//
//       Serial.print("Current depth = ");
//    Serial.println(Current_depth);

    
    }


  }

}
