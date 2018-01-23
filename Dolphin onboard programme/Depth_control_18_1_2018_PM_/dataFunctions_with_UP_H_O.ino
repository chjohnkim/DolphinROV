//===========================================================================
//This method prints some of the variables to the serial monitor for debugging
//===========================================================================
void printData() {
  //For excel data acquisitions
  //  Serial.print("DATA,TIME,");Serial.print(eulX);Serial.print(",");Serial.print(eulY);Serial.print(",");Serial.println(eulZ);



  //  //For normal Operations
  //  Serial.print("    ");Serial.print("Euler Error:(");
  Serial.print(eulX); 
  Serial.print(" "); 
  Serial.print(eulY); 
  Serial.print(" "); 
  Serial.print(eulZ);
  //  Serial.print(" )");

  //  Serial.print("    PWM:");
//  Serial.print("   ");
//  Serial.print(feedMotor1); 
//  Serial.print("    "); 
//  Serial.print(feedMotor2); 
//  Serial.print("    "); 
//  Serial.print(feedMotor3); 
//  Serial.print("    "); 
//  Serial.print(feedMotor4);
//  Serial.print("   Gain:"); 
//  Serial.print(xPropGain);

  Serial.print("  Q1:"); 
  Serial.print(qRef1);
  Serial.print("  Q2:"); 
  Serial.print(qRef2);
  Serial.print("  Q3:"); 
  Serial.print(qRef3);
  Serial.print("  Q4:"); 
  Serial.print(qRef4);

  Serial.print(" Desired_Depth:");
  Serial.print(Desire_depth*10);
  Serial.print(" mm");
  Serial.print(" Current_Depth:");
  Serial.print(Current_depth*10);
  Serial.print(" mm");

  Serial.print(" Thrust:");
  Serial.print(thrust_input);

  Serial.print(" Milliseconds:");
  Serial.print(millis());

  Serial.println();


}

//==========================================================================
//Implement PID Control
//==========================================================================
void pidControl()
{
  //Calculate Proportional Control============================================
  xPropOutput = eulX * xPropGain;
  yPropOutput = eulY * yPropGain;
  zPropOutput = eulZ * zPropGain;

  //Calculate Integral Control================================================
  xIntegralOutput += xIntegralGain * eulX;
  yIntegralOutput += yIntegralGain * eulY;
  zIntegralOutput += zIntegralGain * eulZ;

  //Caculate Derivative Control===============================================
  xDerivativeOutput = xDerivativeGain * (eulX - eulXPrev);
  eulXPrev = eulX;
  yDerivativeOutput = yDerivativeGain * (eulY - eulYPrev);
  eulYPrev = eulY;
  zDerivativeOutput = zDerivativeGain * (eulZ - eulZPrev);
  eulZPrev = eulZ;

  //Caculate total control output from PID
  if (eulX > 2 | eulX < -2) // Deadband
  {
    xOutput = xPropOutput;
  }

  if (eulY > 2 | eulY < -2) // Deadband
  {
    yOutput = yPropOutput;
  }

  if (eulZ > 2 | eulZ < -2) // Deadband
  {
    zOutput = zPropOutput;
  }
}

//==========================================================================
// Feed Appropriate pulses to ESC's
//==========================================================================
void feedMotorPulse() {
  int lowPulse = 1476;
  int highPulse = 14086;
  //X CONFIGURATION

  feedMotor1 = motorStop1 + yOutput - xOutput - thrust_input;// - zOutput; // ADJUST THIS FOR QUADROTOR DYNAMICS ADUSTMENT
  //  /if (feedMotor1 < 1481) feedMotor1 = feedMotor1 - (1481 - feedMotor1) * propellerConstant; // Accounts for the reverse thrust of boat propeller
  if (feedMotor1 > highPulse && feedMotor1 < 1506) feedMotor1 = 1506; // This part of the code makes sure there is no delay for the motor to start moving
  if (feedMotor1 < lowPulse && feedMotor1 > 1457) feedMotor1 = 1457;  // ^^ This depends on the behavior of the esc and motor
  if (feedMotor1 > maxPulse1) feedMotor1 = maxPulse1; // Makes sure that the pulse doesnt go beyond max pwm (2000)
  if (feedMotor1 < minPulse1) feedMotor1 = minPulse1; // Makes sure that the pulse doesnt go beyond min pwm (1000)


  feedMotor2 = motorStop2 - yOutput - zOutput - thrust_input;//- xOutput
  //  /if (feedMotor2 > 1481) feedMotor2 = feedMotor2 + (feedMotor2 - 1481) * propellerConstant; // Accounts for the reverse thrust of boat propeller
  if (feedMotor2 > highPulse && feedMotor2 < 1506) feedMotor2 = 1506; // This part of the code makes sure there is no delay for the motor to start moving
  if (feedMotor2 < lowPulse && feedMotor2 > 1457) feedMotor2 = 1457;  // ^^ This depends on the behavior of the esc and motor
  if (feedMotor2 > maxPulse2) feedMotor2 = maxPulse2; // Makes sure that the pulse doesnt go beyond max pwm (2000)
  if (feedMotor2 < minPulse2) feedMotor2 = minPulse2; // Makes sure that the pulse doesnt go beyond min pwm (1000)

  feedMotor3 = motorStop3 - yOutput - xOutput + thrust_input; //+ zOutput;
  // / if (feedMotor3 < 1481) feedMotor3 = feedMotor3 - (1481 - feedMotor3) * propellerConstant; // Accounts for the reverse thrust of boat propeller
  if (feedMotor3 > highPulse && feedMotor3 < 1506) feedMotor3 = 1506; // This part of the code makes sure there is no delay for the motor to start moving
  if (feedMotor3 < lowPulse && feedMotor3 > 1457) feedMotor3 = 1457;  // ^^ This depends on the behavior of the esc and motor
  if (feedMotor3 > maxPulse3) feedMotor3 = maxPulse3; // Makes sure that the pulse doesnt go beyond max pwm (2000)
  if (feedMotor3 < minPulse3) feedMotor3 = minPulse3; // Makes sure that the pulse doesnt go beyond min pwm (1000)

  feedMotor4 = motorStop4 - yOutput  + zOutput - thrust_input;//+ xOutput
  // /if (feedMotor4 > 1481) feedMotor4 = feedMotor4 + (feedMotor4 - 1481) * propellerConstant; // Accounts for the reverse thrust of boat propeller
  if (feedMotor4 > highPulse && feedMotor4 < 1506) feedMotor4 = 1506; // This part of the code makes sure there is no delay for the motor to start moving
  if (feedMotor4 < lowPulse && feedMotor4 > 1457) feedMotor4 = 1457;  // ^^ This depends on the behavior of the esc and motor
  if (feedMotor4 > maxPulse4) feedMotor4 = maxPulse4; // Makes sure that the pulse doesnt go beyond max pwm (2000)
  if (feedMotor4 < minPulse4) feedMotor4 = minPulse4; // Makes sure that the pulse doesnt go beyond min pwm (1000)

  /////////////////////////////////////////////////////////////////////
  motor1.writeMicroseconds(feedMotor1); //top-left, CW
  motor2.writeMicroseconds(feedMotor2); //top-right, CCW
  motor3.writeMicroseconds(feedMotor3); //bottom-right, CW
  motor4.writeMicroseconds(feedMotor4); //bottom-left, CCW



}
//==========================================================================
//This method caculates the relative error between the desired and actual (qErr=qRef x qMeasCon)
//==========================================================================
void qError() {
  qMeasCon1 = qMeas1;
  qMeasCon2 = -1 * qMeas2;
  qMeasCon3 = -1 * qMeas3;
  qMeasCon4 = -1 * qMeas4;

  qErr1 = (qRef1 * qMeasCon1 - qRef2 * qMeasCon2 - qRef3 * qMeasCon3 - qRef4 * qMeasCon4);
  qErr2 = (qRef1 * qMeasCon2 + qRef2 * qMeasCon1 + qRef3 * qMeasCon4 - qRef4 * qMeasCon3);
  qErr3 = (qRef1 * qMeasCon3 - qRef2 * qMeasCon4 + qRef3 * qMeasCon1 + qRef4 * qMeasCon2);
  qErr4 = (qRef1 * qMeasCon4 + qRef2 * qMeasCon3 - qRef3 * qMeasCon2 + qRef4 * qMeasCon1);
  // return 0;
}

//==========================================================================
//This method caculates the relative error with respect to home configuration (qErrHome = qRefCon x qErr x qRef)
//==========================================================================
void qErrorHome() {
  qRefCon1 = qRef1;
  qRefCon2 = -1 * qRef2;
  qRefCon3 = -1 * qRef3;
  qRefCon4 = -1 * qRef4;

  temp1 = (qErr1 * qRef1 - qErr2 * qRef2 - qErr3 * qRef3 - qErr4 * qRef4);
  temp2 = (qErr1 * qRef2 + qErr2 * qRef1 + qErr3 * qRef4 - qErr4 * qRef3);
  temp3 = (qErr1 * qRef3 - qErr2 * qRef4 + qErr3 * qRef1 + qErr4 * qRef2);
  temp4 = (qErr1 * qRef4 + qErr2 * qRef3 - qErr3 * qRef2 + qErr4 * qRef1);

  qErrHome1 = (qRefCon1 * temp1 - qRefCon2 * temp2 - qRefCon3 * temp3 - qRefCon4 * temp4);
  qErrHome2 = (qRefCon1 * temp2 + qRefCon2 * temp1 + qRefCon3 * temp4 - qRefCon4 * temp3);
  qErrHome3 = (qRefCon1 * temp3 - qRefCon2 * temp4 + qRefCon3 * temp1 + qRefCon4 * temp2);
  qErrHome4 = (qRefCon1 * temp4 + qRefCon2 * temp3 - qRefCon3 * temp2 + qRefCon4 * temp1);

}

//==========================================================================
//This converts the relative quaternion to euler angles
//==========================================================================
void euler() {
  eulZ = atan2(2.0 * (qErrHome2 * qErrHome3 + qErrHome4 * qErrHome1), (sq(qErrHome2) - sq(qErrHome3) - sq(qErrHome4) + sq(qErrHome1)));
  eulY = asin(-2.0 * (qErrHome2 * qErrHome4 - qErrHome1 * qErrHome3) / (sq(qErrHome2) + sq(qErrHome3) + sq(qErrHome4) + sq(qErrHome1)));
  eulX = atan2(2.0 * (qErrHome3 * qErrHome4 + qErrHome1 * qErrHome2), (-sq(qErrHome2) - sq(qErrHome3) + sq(qErrHome4) + sq(qErrHome1)));
  eulZ = eulZ * 180 / 3.1415;
  eulY = eulY * 180 / 3.1415;
  eulX = eulX * 180 / 3.1415;
}

//==========================================================================
//This filters measured unit quaternions if the length is less than 0.99
//==========================================================================

void filter() {
  qLength = sqrt(sq(qMeas1) + sq(qMeas2) + sq(qMeas3) + sq(qMeas4));
  if (qLength < 0.97) {
    qMeas1 = qMeasPrev1;
    qMeas2 = qMeasPrev2;
    qMeas3 = qMeasPrev3;
    qMeas4 = qMeasPrev4;

  }
}

//==========================================================================
//This receives a trigger from the user via serial monitor to change the desired orientation (enter 1 into the serial monitor to trigger)
//==========================================================================
/* read the user input
 p: increase proportional gain by 0.1
 o: decrease proprtional gain by 0.1
 */
void trigger()
{
  if (Serial.available() > 0)
  {
    rx_byte = Serial.read();
    switch (rx_byte)
    {
    case '1':
      qRef1 = qMeas1;
      qRef2 = qMeas2;
      qRef3 = qMeas3;
      qRef4 = qMeas4;
      break;
    case '0':
      s = 0;
      break;

    case 'p':
      xPropGain = xPropGain + 0.1;
      yPropGain = yPropGain + 0.1;
      zPropGain = zPropGain + 0.1;
      break;

    case 'o':
      xPropGain = xPropGain - 0.1;
      yPropGain = yPropGain - 0.1;
      zPropGain = zPropGain - 0.1;
      break;
    }
  }
}

void update_home_orientation_and_thrust_depth()
{
  int inByte = 0;
  char buf[256];
  char stopByte = '@';
//  int Q_recv, T_recv, Depth_recv;
//  float Q1, Q2, Q3, Q4;
//
//  float Thrust_gain = 0.5;
//  float Depth_gain = 0.5


    int i = 0;
  
 
  while (true) {
    
    while (Serial.available() > 0) { 
      inByte = Serial.read();

      if (inByte == stopByte) {
        buf[i] = '\0';
        break;
      }
      buf[i] = inByte;
      i++;
    }

    break;
  }

  char prefix = buf[0];

  int k = 1;

  while (buf[k] != '\0') {
    buf[k - 1] = buf[k];
    k++;
  }

  buf[k - 1] = '\0';

  if(prefix=='A')
  {
    // Q1
    Q_recv = atof(buf);
    Q1 = (float)Q_recv / 100.0;
    qRef1 = Q1;

  }
  else if(prefix=='B'){

    Q_recv = atof(buf);
    Q2 = (float)Q_recv / 100.0;
    qRef2 = Q2;

  }
  else if(prefix=='C'){

    Q_recv = atof(buf);
    Q3 = (float)Q_recv / 100.0;
    qRef3 = Q3;

  }
  else if(prefix=='D'){

    Q_recv = atof(buf);
    Q4 = (float)Q_recv / 100.0;
    qRef4 = Q4;

  }
  else if(prefix=='T'){

    T_recv = atof(buf);

    thrust_input = T_recv*Thrust_gain;

  }
  else if(prefix=='E'){

    qRef1 = Depth_con_qRef1;
    qRef2 = Depth_con_qRef2;
    qRef3 = Depth_con_qRef3;
    qRef4 = Depth_con_qRef4;

    Depth_recv = atof(buf);
    Desire_depth = ((float)Depth_recv * Depth_recv_gain) / 10.0;
    
    Current_depth = sensor.getPressure(ADC_4096) - 1000.0;

    thrust_input = (Desire_depth - Current_depth)*Depth_thrust_gain;

    Serial.print("Thrust input = ");
    Serial.println(thrust_input);

  }

}

void simple_depth_control()
{

    qRef1 = Depth_con_qRef1;
    qRef2 = Depth_con_qRef2;
    qRef3 = Depth_con_qRef3;
    qRef4 = Depth_con_qRef4;

  int inByte = 0;
  char buf[256];
  char stopByte = '@';

 int i = 0;
 while (true) {
    
    while (Serial.available() > 0) { 
      inByte = Serial.read();

      if (inByte == stopByte) {
        buf[i] = '\0';
        break;
      }
      buf[i] = inByte;
      i++;
    }

    break;
  }

  char prefix = buf[0];

  int k = 1;

  while (buf[k] != '\0') {
    buf[k - 1] = buf[k];
    k++;
  }

  buf[k - 1] = '\0';

  if(prefix=='D')
  {
    // Set desire depth
     Depth_recv = atof(buf);
    Desire_depth = ((float)Depth_recv * Depth_recv_gain) / 10.0;
    Serial.print("Desire depth = ");
    Serial.print(Depth_recv);
    Serial.println("mm");
    Serial.print("Current depth = ");
    Serial.print(Current_depth*10);
    Serial.println("mm");

  }

    Current_depth = sensor.getPressure(ADC_4096) - 1000.0;

    if((Desire_depth - 0.2)<Current_depth<(Desire_depth+0.2))
    {
     
    qRef1 = Depth_con_qRef1;
    qRef2 = Depth_con_qRef2;
    qRef3 = Depth_con_qRef3;
    qRef4 = Depth_con_qRef4;
      
      
      thrust_input = 0;
    }

   if((Desire_depth - Desire_depth_range)>Current_depth)
    {
      if(T1 == 1){
      qRef2 = Depth_con_tilt_up;
     
      // qRef2 = (Desire_depth - Current_depth)*Depth_con_tilt_gain;

//        if(qRef2 > 0.3){
//          
//          qRef2 = 0.3;
//        
//        }
        
      }

      if(T2 == 1){
    thrust_input = (Desire_depth - Current_depth)*Depth_thrust_gain;
      }
      
    }

    if((Desire_depth + Desire_depth_range)<Current_depth)
    {
      
      if(T1 == 1){
      qRef2 = Depth_con_tilt_down;
     
     //   qRef2 = (Desire_depth - Current_depth)*Depth_con_tilt_gain;

//        if(qRef2 < -0.3){
//          
//          qRef2 = -0.3;
//        
//        }
      }
      
       if(T2 == 1){
    thrust_input = (Current_depth - Desire_depth)*Depth_thrust_gain;
       }
       
    }
  
}

void initialized_the_motor()
{

  if(Serial.available()){

    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {


      case 'w':
        for(float i = 0 ; i < 20 ; i=i+0.5 ){
          motor1.write(motorStop1+i); 
          motor2.write(motorStop1+i);
          motor3.write(motorStop1+i); 
          motor4.write(motorStop1+i); 
          delay(100);
        }    
        //  Serial.println("Forward");
        break;

      case 'x':
        for(float i = 0 ; i < 20 ; i=i+0.5 ){
          motor1.write(motorStop1-i); 
          motor2.write(motorStop1-i);
          motor3.write(motorStop1-i); 
          motor4.write(motorStop1-i); 
          delay(100);
        }           
        // Serial.println("Backward");
        break;


      }
    }
  }

}

void update_depth_info()
{

  Current_depth = sensor.getPressure(ADC_4096) - 1000.0;
}











