# DolphinROV
Dolphin ROV project

  Ocean is a very mysterious place. In the past, different kind of underwater robots had been developed for exploring the ocean. However, those robots are all bulky and clumsy. The aim of the Dolphin ROV project is to develope a highly agile and dynamic underwater robot for ocean exploration or other underwater application. 
  
    Inside the Dolphin V4X mark III, there is a Raspberry Pi computer and an Arduino Nano microcontroller. The Raspberry Pi acts as a central communication hub which handle all the data traffic between PC, camera and Arduino. The Arduino Nano is the controller of the robot, all the Attitude , Dynamic and Depth Control are written in the Arduino code and the code is run by the Arduino Nano board.
    
    This Github store all the version of the Arduino code for the Dolphin ROV project
    
    How to USE the Arduino code
    
    The latest Arduino code is "Turning control 22 March 2018" , below we explain how to operate the robot with this latest code.
    
    1. On your PC, use remote desktop (eg. VNC server) to login to the Raspberry Pi
    
    2. In the Raspberry Pi desktop, open the Arduino programme and upload the "Turning control 22 March 2018" code to the Arduino Nano
    
    3. Open the serial monitor
    
    4. Input "d" to check the current water depth and the desired depth
    
    5. Input "9" to activate depth control
    
    6. Input "w" to initiate the thruster
    
    7. Input "s" to start the experiment
    
    8. Input "0" to STOP the robot
