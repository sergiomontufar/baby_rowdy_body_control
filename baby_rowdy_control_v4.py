'''

#UTSA
#Sergio Montufar Research Project
Spring 2024
v1.0
March 12 2024

v2.0
March 15 2024
Use of a global array for the motors
Use of arrays for the positions
v3.0
March 16 2024
incorporate MQTT
incorporate the set_ang_position(position) function. It goes directly to that position in cartesian plane 
v4.0
March 22 2024
Introduce the trajectory_XYZ function, to go to the desired position in XYZ through a 100 points trajectory in cartesian plane
Introduce the trajectory_ang function, to go to the desired position in XYZ through a 100 points trajectory in angular space
Introduce the uo_down function, using 100 point of angular increments
v6.0
March 23 2024
This program assumes Initial position VERTICAL!!
INITIAL POSITION of the robot MUST be Completely vertical!! (Requires the support)
    - adjust direct_kinematics and inverse_kinematics to acomplish that
    - The outputs of these routines are then modified to be consistent with the robot motors. this was tested using  debug =1 in each function under test
    
Add debug variable to control how many debug prints are executed. 
    debug vaiable is set just before each call to main functions in the main control routine
    - debug = 0 removes all prints and allow sending movements to the motors
    - debug = 1 prints seversl messages but does NOT send commands to the motors
 
 Working functions:
    - Home
    - reset
    - up_down   
    - TrajectoryXYZ   For now it uses an interpolation in the ANGULAR SPACE, NOT in the Cartesian space

v7.0
March 24 2024
Correct return to home function
Correct direct_kinematics and inverse_kinematic fuctions 
INITIAL POSITION of the robot MUST be Completely vertical!! (Requires the support)
    - direct_kinematics and inverse_kinematics to acomplish that
    
 Working functions:
    - Home
    - reset
    - sinusoidal_up_down   
    - TrajectoryXYZ   For now it uses an interpolation in the ANGULAR SPACE, NOT in the Cartesian space
    - sinusoidal step
v8.0
April 5 2024
 New Working functions:
    - sinusoidal side_step
    - display_ang_currents
    - display_ang_torques
    
    Publish angles on topic body/monitor/angles
    Publish torques on topic body/monitor/torques
    Publish currents on topic body/monitor/currents
    
v9.0
April 6 2024    
First test in floor
    - Increase the PD  P gin a lot!
    - Include OVER current protection MAX_CURRENT = 14
    - It walks backwards!
    - Modified the walker support to let the robot go up-down freely
    - Modify User interface to siplay aal the curents
    
v10.0
April 12 2024  
Publish total current
publish all the variables every 500 mSeg
  
  
  
v11.0
April 14-21 2024  
Modify the main program to be a low level control:
    - every 10 mSeg is reading variables and sending positions to the motors
    - every 100 mSeg is publishing variables
    - the motor positions come from upper level equilibrium control or
    - the motor positions come from upper level trayectory control
 Working functions with the new low level control (every 10 mSecs) :
    - Trajectory_XYZ
    - Home
    - Reset
    - up_down
    
    
v12.0
April 21 2024  
 Working functions with the new low level control (every 10 mSecs) :
    - sinusolidal_step
    - sinusolidal_side_step

v13.0
April 23 2024 
    - This is the first time it stands at home  stable, consuming only 4 Amps total!
    
    
v14.0
April 24 2024 

v15.0
April 30 2024 
    - modification to step routine with 3 different sinusoidals with a delay between them
    May 7 2024
    -walk parameters now can be adjusted from the GUI
    
v16.0
May 9 2024 
    - home parameters now can be adjusted from the GUI

    
    
baby_rowdy_control_v1

The new baby Rowdy robot has now Top body!

V1.0
July 7, 2024
    - Incorporate 3 motors of top body 
V1.1
July 9, 2024
    - Incorporate set points for each joint from User Interface via MQTT       
    
    
V2
July 25, 2024
    - Incorporate wheels control via MQTT on the UI (Visual Basic)
    - but this program DO NOT contol the Wheels!

    - A new program is generated (rollie_wheels_control v1.0 ) to control only the wheels. This is beause the wheels control slowers the legs control loop


v4
August 7, 2024
    - Improved MQTT SETUP configuration, to be equal to the other programs connecting to mosquitto
    - include anckles in Forward_step:  See program baby_rowdy_half_step_v2 in main computer for plotting the sinusoidal curves
    - The type of step is defined by the following parameters: 
        half_size_step = 2   #2 = complete step of two legs moving
        CM_frequency_factor = 1   
        
      - Add Jogging_step function both in raspberry and in VB Graphical User Interface
      - MAXCURRENT Protection 
            + The protection was NOT being executed!!
            + Now it is executed in the finction read_variables()  every 10MSec
            + Set to: MAX_CURRENT = 10.0
        
To do..   
    - adition to sequential home roitine, one leg a a time
    - Equilibrium functions implemented
    
    
'''
import sys
import matplotlib.pyplot as plt
from time import sleep
from random import uniform

import numpy as np
import time
import math
from TMotorCANControl.mit_can import TMotorManager_mit_can

from witmotion import IMU

##########################################################
# IMU setup
RPY_angles = [0.0, 0.0, 0.0]  # Initializing as a list with three float values

def callback_IMU(msg):
    global RPY_angles
    
    #print(msg)
    imu_string= imu.get_angle()
    #print (imu_string)
    #imu.get_angular_velocity()
    #imu.get_acceleration()
    

    roll, pitch, jaw = imu_string
    #print (f"Roll = {roll}, Pitch = {pitch}, Jaw = {jaw}")
    RPY_angles = roll, pitch, jaw 
    #print(RPY_angles)
    
imu = IMU()
imu.subscribe(callback_IMU)





VERSION = "baby_rowdy_control version 4.0"


##########################################################
# Motor CAN addresses
#angles_list = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"] 

ID_1 = 3   # left lower_leg
ID_2 = 2  # left upper_leg
ID_3 = 6    #left pitch
ID_4 = 8    #left jaw

ID_5 = 5    # Right lower_leg
ID_6 = 4  # right upper_leg
ID_7 = 7  #right pitch
ID_8 = 9   #right jaw


ID_9 = 10  # neck
ID_10 = 11  #right anckle
ID_11 = 12  #left anckle



Type_1 = 'AK60-6'
    
state = False
debug = False

MAX_CURRENT = 10.0




theta2_zero = 0.0
theta3_zero = 0.0
theta4_zero = 0.0
theta5_zero = 0.0
theta6_zero = 0.0
theta7_zero = 0.0
theta8_zero = 0.0   
theta9_zero = 0.0
theta10_zero = 0.0
theta11_zero = 0.0   
theta12_zero = 0.0 


ZERO_POSITIONS = [theta3_zero,theta2_zero,theta6_zero,theta8_zero,theta5_zero,theta4_zero,theta7_zero,theta9_zero,theta10_zero,theta11_zero,theta12_zero]

# home position angles in degrees
home_upper_leg_ang = 18
home_lower_leg_ang = 30
home_CM_displacement_ang  = 10
home_Turn_ang = 0
home_step_neck_ang = 25

#home position angles in radians
angles_list = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"] 
offset3 = np.radians(home_lower_leg_ang)  #left lower leg m3
offset2 = np.radians(home_upper_leg_ang)   #left upper leg m2
offset6 = np.radians(home_CM_displacement_ang)
offset8 = np.radians(home_Turn_ang)

offset5 = -np.radians(home_lower_leg_ang)  #right lower leg m5
offset4 = -np.radians(home_upper_leg_ang) #right upper leg m4
offset7 = -np.radians(home_CM_displacement_ang)
offset9 = np.radians(home_Turn_ang)

offset10 = np.radians(home_step_neck_ang)  # neck
offset11 = 0.0 # right ankle
offset12 = 0.0 # left ankle

INI_POSITIONS = [ offset3, offset2, offset6, offset8, offset5, offset4, offset7, offset9, offset10, offset11, offset12]
 

amplitude2 = 1.0
amplitude3 = 1.0
amplitude6 = 1.0
amplitude8 = 1.0  
amplitude12 = 1.0 

amplitude4 = -1.0
amplitude5 = -1.0
amplitude7 = -1.0
amplitude9 = -1.0 
amplitude11 = -1.0 

amplitude10 = 1.0 


amplitudes = [amplitude3,amplitude2,amplitude6,amplitude8,amplitude5,amplitude4,amplitude7,amplitude9,amplitude10,amplitude11,amplitude12]
UP_DOWN_enable_joints = [1.0,1.0,0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0]

# Robot leg's dimensions

L0 = 4.623258381
L1 = 5.411252782
L2 = 5.411252782



    
# defaut step parameters
main_period =    2.0  # This is the period of the walking sequence 

step_upper_leg_ang = 10.0
step_lower_leg_ang = 30.0
step_CM_displacement_ang = 15.0
step_turn_ang = 0.0
step_neck_ang = 0.0
step_ankle_ang = 0.0

step_delay_sin_lower_leg = main_period/16 
step_delay_sin_upper_leg = main_period/8 
step_delay_sin_anckle = main_period/6 
step_delay_sin_neck = main_period/2 

#Step type
half_size_step = 2  # 2 = complete step of two legs moving
CM_frequency_factor = 1   

STEP_amplitudes = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_neck_ang,step_ankle_ang,step_ankle_ang]
STEP_amplitudes = np.radians(STEP_amplitudes)

jog_upper_leg_ang = 35.0
jog_lower_leg_ang = 35.0
jog_CM_displacement_ang = 20.0
jog_turn_ang = 0.0
jog_neck_ang = 0.0
jog_ankle_ang = 0.0

jog_delay_sin_lower_leg = main_period/8 
jog_delay_sin_upper_leg = main_period/8 
jog_delay_sin_anckle = main_period/6 
jog_delay_sin_neck = main_period/2 

Jogging_amplitudes = [jog_lower_leg_ang,jog_upper_leg_ang,jog_CM_displacement_ang,jog_turn_ang,jog_lower_leg_ang,jog_upper_leg_ang,jog_CM_displacement_ang,jog_turn_ang,jog_neck_ang,jog_ankle_ang,jog_ankle_ang]
Jogging_amplitudes = np.radians(Jogging_amplitudes)


SIDE_STEP_amplitudes = [0.5,0.5,-0.3,0.0,0.5,0.5,-0.3,0.0,0.0,0.0,0.0]

new_ang_position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
old_ang_position  = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
variables_dict = {}
new_XYZ_position = [0.0,0.0,0.0]
old_XYZ_position = [0.0,0.0,0.0]
XYZ_dict ={}
RPY_dict={}

motor_currents = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
motor_torques = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
thetas_up_down = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

command = ""
trajectory_finished = True  #initial condition
trajectory = []
traj_idx = 0
num_cycles = 1
local_cycles = 0
direction = 1  # Direction of traversal (1 for increasing, -1 for decreasing)
idx_theta = 0

np_ini_pos = np.array(INI_POSITIONS)
np_zero_pos = np.array(ZERO_POSITIONS)
np_amplitudes= np.array(amplitudes)

motors = [] 
motors_dict = {}

positions_anterior = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]


##########################################################
# MQTT setup
import paho.mqtt.client as mqtt
import ssl
import json


############################################################################
# MQTT Parameters & Functions

#serverAddress = "localhost"
#serverAddress = "127.0.0.1"
serverAddress = "rollie-body-pi"
#serverAddress = "baby-body-pi"
#serverAddress = "192.168.8.7"

# serverAddress, is the pi's host name. But, since our Mosquitto broker and
# this program (which acts as the subscriber) are on the same Raspberry Pi
# we can simply use "localhost" as the server name.

############################################################################
# MQTT Functions
# serverAddress, below is your pi's host name. But, since our Mosquitto broker and
# this program (which acts as the subscriber) are on the same Raspberry Pi
# we can simply use "localhost" as the server name.



def on_connect(client, userdata, flags, rc):
    global didPrintSubscribeMessage
    if not didPrintSubscribeMessage:
        didPrintSubscribeMessage = True
        print("subscribing")
        
        mqttClient.subscribe([ ("body/control/positionXYZ", 1), ("body/control/TrajectoryXYZ", 1), ("body/control/Set_angles", 1),("body/control/command", 1),("body/control/monitor", 1), ("body/control/exit", 1), ("body/control/walk_parameters" , 1), ("body/control/home_parameters" , 1)])

        print("subscribed")
        print("Connected to Mosquitto result code "+str(rc))
 # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #AWS
    # client.subscribe([("$aws/things/raspberryPi4/shadow/update", 1), ("$aws/things/raspberryPi4/shadow/get", 1)])
    
def on_message(client, userdata, message):
    
 
    global new_XYZ_position
    global desired_ang_position
    global debug
    global salir
    global command
    global X,Y,Z
    global num_cycles

    global STEP_amplitudes
    
    global home_upper_leg_ang
    global home_lower_leg_ang 
    global home_CM_displacement_ang 
    global home_Turn_ang 
    


    
    #print("Message received: " + message.topic + " : " + str(message.payload))
    #print("topic :",message.topic)
    #print(type(message.topic))      
    
    #print("payload :",message.payload)
    #print(type(message.payload)) 
       
    m_decode=str(message.payload.decode("utf-8","ignore"))
    #print("data Received type",type(m_decode))
    print("data Received",m_decode)
     
    #if message.topic == '$aws/things/raspberryPi4/shadow/update':
    if message.topic == 'body/control/positionXYZ' or message.topic == 'body/control/TrajectoryXYZ'  :    
        
        if message.topic == 'body/control/positionXYZ':
            print("body/control/positionXYZ recibido")
            command = "positionXYZ"
        if message.topic == 'body/control/TrajectoryXYZ':
            print("body/control/TrajectoryXYZ recibido")
            command = "TrajectoryXYZ"
        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'desired' in m_decode:    
            X = int((m_in["state"]["desired"]["X"]))
            
            Y = int((m_in["state"]["desired"]["Y"]))
            
            Z = int((m_in["state"]["desired"]["Z"]))
            
            
            new_XYZ_position = [X,Y,Z]

            
            if debug >= 1:
                print(" //// Desired position //// ")
                print("\n new_XYZ_position = ", new_XYZ_position)
                
    elif message.topic == 'body/control/Set_angles'  :    
        

        print("body/control/Set_angles recibido")
        command = "Set_angles"

        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'desired' in m_decode:    
            Theta2 = float((m_in["state"]["desired"]["Theta2"]))
            Theta3 = float((m_in["state"]["desired"]["Theta3"]))
            Theta4 = float((m_in["state"]["desired"]["Theta4"]))
            Theta5 = float((m_in["state"]["desired"]["Theta5"]))
            Theta6 = float((m_in["state"]["desired"]["Theta6"]))
            Theta7 = float((m_in["state"]["desired"]["Theta7"]))
            Theta8 = float((m_in["state"]["desired"]["Theta8"]))
            Theta9 = float((m_in["state"]["desired"]["Theta9"]))
            Theta10 = float((m_in["state"]["desired"]["Theta10"]))
            Theta11 = float((m_in["state"]["desired"]["Theta11"]))
            Theta12 = float((m_in["state"]["desired"]["Theta12"]))
            
            
    
            
            
            desired_ang_position_deg = [ Theta3 ,  Theta2 ,  Theta6 ,  Theta8 ,  Theta5 ,  Theta4 ,  Theta7 ,  Theta9 ,  Theta10 ,  Theta11 ,  Theta12 ] 

            desired_ang_position =  [math.radians(angle) for angle in desired_ang_position_deg]
            
            if debug >= 1:
                print(" //// Desired angular positions //// ")
                print("\n desired_ang_position [deg] = ", desired_ang_position_deg)
                
            
                             
    elif message.topic =="body/control/walk_parameters"   :    
        
        print("body/control/walk_parameters recibido")
        command = "walk_parameters"
        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'parameters' in m_decode:    
            
            step_upper_leg_ang = float((m_in["step"]["parameters"]["Upper_Leg_Ang"]))
            
            step_lower_leg_ang = float((m_in["step"]["parameters"]["Lower_Leg_Ang"]))
            
            step_CM_displacement_ang = float((m_in["step"]["parameters"]["CM_disp_Ang"]))
 

            
            step_delay_sin_lower_leg = float((m_in["step"]["parameters"]["Delay_Lower_leg"]))
            
            step_delay_sin_upper_leg = float((m_in["step"]["parameters"]["Delay_Upper_leg"]))
            
            half_size_step = int((m_in["step"]["parameters"]["Half_size_step"]))
            
            step_turn_ang = float((m_in["step"]["parameters"]["Turn_Ang"]))
            
            step_delay_sin_neck = float((m_in["step"]["parameters"]["Delay_Neck"]))
            delay_sin_ankle = float((m_in["step"]["parameters"]["Delay_Ankle"]))           
            step_neck_ang =  float((m_in["step"]["parameters"]["Neck_ang"]))
            step_ankle_ang = float((m_in["step"]["parameters"]["Ankle_ang"]))    
            
            main_period = int((m_in["step"]["parameters"]["Main_period"]))     
              

            print(" //// step parameters: //// ")
            print("\r Upper_Leg_Ang = ", step_upper_leg_ang)
            print("\r Lower_Leg_Ang = ", step_lower_leg_ang)
            print("\r CM_disp_Ang = ", step_CM_displacement_ang)
            print("\r Main_period = ", main_period)
            print("\r Delay_Lower_leg = ", step_delay_sin_lower_leg)
            print("\r Delay_Upper_leg = ", step_delay_sin_upper_leg)
            print("\r Half_size_step = ", half_size_step)
            print("\r Turn_Ang = ", step_turn_ang)
            print("\r Delay_Neck = ", step_delay_sin_neck)
            print("\r Delay_Ankle = ", delay_sin_ankle)
            print("\r step_neck_ang = ", step_neck_ang)
            print("\r step_ankle_ang = ", step_ankle_ang)

            STEP_amplitudes = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_neck_ang,step_ankle_ang,step_ankle_ang]
            STEP_amplitudes = np.radians(STEP_amplitudes)
            
            
           
                
    elif message.topic =="body/control/home_parameters"   :    
        
        print("body/control/walk_parameters recibido")
        command = "walk_parameters"
        #print("Converting from Json to dictionary Object")
        m_in=json.loads(m_decode) #decode json data
        #print(type(m_in))
        #print(m_in)
        #print("Print each key-value pair from JSON response")
        #for key, value in m_in.items():
        #    print(key, ":", value)
        #    print('\n')

        if 'parameters' in m_decode:    
            home_upper_leg_ang = float((m_in["home"]["parameters"]["home_Upper_Leg_Ang"]))
            
            home_lower_leg_ang = float((m_in["home"]["parameters"]["home_Lower_Leg_Ang"]))
            
            home_CM_displacement_ang = float((m_in["home"]["parameters"]["home_CM_disp_Ang"]))
 
            home_Turn_ang = int((m_in["home"]["parameters"]["home_Turn_Ang"]))
            

  

            print(" //// step parameters: //// ")
            print("\r home_Upper_Leg_Ang = ", home_upper_leg_ang)
            print("\r home_Lower_Leg_Ang = ", home_lower_leg_ang)
            print("\r home_CM_disp_Ang = ", home_CM_displacement_ang)
            print("\r home_Turn_Ang = ", home_Turn_ang)

            update_home_positions()
                           
    elif  message.topic == 'body/control/command':       
        
        
        if 'home' in m_decode:   #direct command, its not json data! 
            print("body/control/command  home recibido") 
            command = "home" 
        elif 'reset' in m_decode:    
            print("body/control/command  reset recibido") 
            command = "reset"   
        elif  'Forward_step' in m_decode:       
            print("body/control/command Forward_step recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'Forward_step' in m_decode:    
                num_cycles = int((m_in["Forward_step"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "Forward_step" 
            setup_sinusolidal_step(1) #cycles must be integer
        elif  'Jogging_step' in m_decode:       
            print("body/control/command Jogging_step recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'Jogging_step' in m_decode:    
                num_cycles = int((m_in["Jogging_step"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "Jogging_step" 
            setup_sinusolidal_Jogging_step(1) #cycles must be integer 
        elif  'side_step' in m_decode:       
            print("body/control/command side_step recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'side_step' in m_decode:    
                num_cycles = int((m_in["side_step"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "side_step"                
        elif  'up_down' in m_decode:       
            print("body/control/command up_down recibido") 
            m_in=json.loads(m_decode) #decode json data
            if 'up_down' in m_decode:    
                num_cycles = int((m_in["up_down"]["parameter"]["cycles"]))
                print("cycles = ",num_cycles)
            command = "up_down" 
            setup_up_down()
    elif  message.topic == 'body/control/monitor':   
          
        if  'angles' in m_decode:       
            print("body/control/monitor_angles recibido") 
            command = "monitor_angles"     
        elif  'XYZ'in m_decode:       
            print("body/control/monitor_XYZ recibido") 
            command = "monitor_XYZ"   
        elif  'RPY'in m_decode:       
            print("body/control/monitor_RPY recibido") 
            command = "monitor_RPY"  
        elif  'torques'in m_decode:       
            print("body/control/monitor_torques recibido") 
            command = "monitor_torques"     
        elif  'currents'in m_decode:       
            print("body/control/monitor_currents recibido") 
            command = "monitor_currents"              
    elif  message.topic == 'body/control/exit':       
        print("body/control/exit recibido") 
         
        salir = True   # al salir se va a Home!
    
# Function to publish message
def publish_message(client, topic, message):
    client.publish(topic, message)
    #print("Message published:", message)




##########################################################
# T-Motors control routines

##################################################################
def update_home_positions():
    global INI_POSITIONS
    global home_upper_leg_ang
    global home_lower_leg_ang 
    global home_CM_displacement_ang 
    global home_Turn_ang 
    global home_step_neck_ang 
    
    offset3 = np.radians(home_lower_leg_ang)  #left lower leg m3
    offset2 = np.radians(home_upper_leg_ang)   #left upper leg m2
    offset6 = np.radians(home_CM_displacement_ang)
    offset8 = np.radians(home_Turn_ang)

    offset5 = -np.radians(home_lower_leg_ang)  #right lower leg m5
    offset4 = -np.radians(home_upper_leg_ang) #right upper leg m4
    offset7 = -np.radians(home_CM_displacement_ang)
    offset9 = -np.radians(home_Turn_ang)

    offset10 = np.radians(home_step_neck_ang)  # neck
    offset11 = 0.0 # right ankle
    offset12 = 0.0 # left ankle

    INI_POSITIONS = [ offset3, offset2, offset6, offset8, offset5, offset4, offset7, offset9, offset10, offset11, offset12]
            
                   
def display_ang_positions():
    global old_ang_position
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0    
    # Initialize an empty dictionary
    angles_list = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
    Thetas_dict = {}
    
    for idx, (key, motor) in enumerate(motors_dict.items()):
 
        #motors[idx].update()   
        #motor.update()
        #time.sleep(0.1)
        #theta = motors[idx].position   # Reporta un estado anterior
        theta = motor.position
        current = motor._motor_state.current
        total_current += current
        old_ang_position[idx] = theta+ ZERO_POSITIONS[idx]
            #Create a dictionary for publish values
        #Thetas_dict[angles_list[idx]] = round(np.degrees(theta),2)
        Thetas_dict[key] = round(np.degrees(theta),2)
        
        if debug >= 3: 
            print("\r  theta", idx, theta,"rads ", np.degrees(theta), "degrees")
            print("\r  Current", idx, current,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
           
        
    
    if debug >= 1:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")   
        print("\r  old_ang_position (rads)", old_ang_position)
            # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(old_ang_position))
            # Join the formatted strings into a single string and print
        print(f"  old_ang_position (degrees): {', '.join(formatted_numbers)}")
        print("\r  ============================================================")
 
        #Publish the angles
    topic = "body/monitor/angles"
    print(Thetas_dict)
        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(Thetas_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
 
 
def display_XYZ_positions():
    global old_ang_position
    global old_XYZ_position
    global debug
    global XYZ_dict
    
    display_ang_positions()
    old_XYZ_position = direct_kinematics(old_ang_position)
    
    if debug >= 3:
        print("\r  ============================================================")
        print("\r  old_XYZ_position", old_XYZ_position)
        print("\r  ============================================================")  
        
    XYZ_dict["X"]=old_XYZ_position[0]
    XYZ_dict["Y"]=old_XYZ_position[1]
    XYZ_dict["Z"]=old_XYZ_position[2]     


            #Publish the XYZ positions
    topic = "body/monitor/XYZ"
    print(XYZ_dict)
        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(XYZ_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
        
def publish_variables():
    global debug
    global variables_dict
    
                #Publish the all variables
    topic = "body/monitor/all"
    if debug >= 2:
        print(variables_dict)
    
        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(variables_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
     
         
def read_variables():
    global old_ang_position
    global old_XYZ_position
    global debug
    global motor_currents
    global variables_dict
    global RPY_angles 
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0    
    # Initialize an empty dictionary
    angles_list = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"]
    currents_list = ["I m3", "I m2", "I m6", "I m8", "I m5", "I m4", "I m7", "I m9", "I m10", "I m11", "I m12", "I tot"]
    torques_list = ["Torque3", "Torque2", "Torque6", "Torque8", "Torque5", "Torque4", "Torque7", "Torque9", "Torque10", "Torque11", "Torque12"]
    variables_dict = {}
    
    for idx, motor in enumerate(motors):
 

        theta = motor.position
        current = abs(motor._motor_state.current)
        
        #Current protection  VERY IMPORTANT!
        if current > MAX_CURRENT:
            print(" \r *********  WARNING !! ***********************************************************")
            print("\r ******  MAX CURRENT EXEEDED !! *****  Motor:",currents_list[idx], "Value: ",current)
            print(" \r *********************************************************************************")
            print("\n")
            sys.exit()
                
                
        torque = motor.torque
        total_current += current
        old_ang_position[idx] = theta+ ZERO_POSITIONS[idx]
            #Create a dictionary for publish values
        variables_dict[angles_list[idx]] = round(np.degrees(theta),2)
        variables_dict[currents_list[idx]] = round(current,2)
        variables_dict[torques_list[idx]] = round(torque,2)

    old_XYZ_position = direct_kinematics(old_ang_position)
    variables_dict["X"]=old_XYZ_position[0]
    variables_dict["Y"]=old_XYZ_position[1]
    variables_dict["Z"]=old_XYZ_position[2] 
        
    variables_dict[ "I tot"] = round(total_current,2)   
    
    variables_dict["Roll"]=RPY_angles[0]
    variables_dict["Pitch"]=RPY_angles[1]
    variables_dict["Yaw"]=RPY_angles[2]     

    
    
    #publish_variables()


    if debug >= 3:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")   
        print("\r  old_ang_position (rads)", old_ang_position)
        print("\r  old_XYZ_position", old_XYZ_position)
        print("\r  ============================================================")      
  
def display_RPY_angles():

    global debug
    global RPY_dict
    global RPY_angles 
    

        
    RPY_dict["Roll"]=RPY_angles[0]
    RPY_dict["Pitch"]=RPY_angles[1]
    RPY_dict["Yaw"]=RPY_angles[2]     

    if debug >= 2:
        #print("\r  ============================================================")
        #print("\r  RPY_angles", RPY_angles)
        print(RPY_dict) 
        #print("\r  ============================================================")  
           
        
            #Publish the torques
    topic = "body/monitor/RPY"

        # Serialize dictionary to JSON string with newline after each key-value pair
    json_string = json.dumps(RPY_dict, indent=4)  # Pretty print with indentation
    json_string_with_newline = '\n'.join(json_string.split('\n'))

        # Publish JSON string to MQTT broker
    publish_message(mqttClient, topic, json_string_with_newline)
        
       
def display_ang_currents():
    global motor_currents
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0   
        # Initialize an empty dictionary
    currents_list = ["I m3", "I m2", "I m6", "I m8", "I m5", "I m4", "I m7", "I m9", "I m10", "I m11", "I m12", "I tot"]
    currents_dict = {} 
    for idx, motor in enumerate(motors):

        current = abs(motor._motor_state.current)
        motor_currents[idx] = current
        if current > MAX_CURRENT:
                print(" \r *********  WARNING !! ***********************************************************")
                print("\r ******  MAX CURRENT EXEEDED !! *****  Motor:",currents_list[idx], "Value: ",current)
                print(" \r *********************************************************************************")
                print("\n")
                sys.exit()
        total_current += current
            #Create a dictionary for publish values
        currents_dict[currents_list[idx]] = round(current,2)
        
        
        if debug >= 3: 
            print("\r  Current", idx, current,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
            
    currents_dict[ "I tot"] = round(total_current,2)
    
    if debug >= 1:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")   

            # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", motor_currents)
            # Join the formatted strings into a single string and print
          
        print(f"  motor_currents (Amps): {', '.join(formatted_numbers)}")
        print("\r  ============================================================")
        
            #Publish the currents
        topic = "body/monitor/currents"
        print(currents_dict)
            # Serialize dictionary to JSON string with newline after each key-value pair
        json_string = json.dumps(currents_dict, indent=4)  # Pretty print with indentation
        json_string_with_newline = '\n'.join(json_string.split('\n'))

            # Publish JSON string to MQTT broker
        publish_message(mqttClient, topic, json_string_with_newline)

       
def protect_max_currents():
    global motor_currents
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
    total_current = 0.0   

    for idx, motor in enumerate(motors):

        current = abs(motor._motor_state.current)
        motor_currents[idx] = current
        if current > MAX_CURRENT:
                print(" \r *********  WARNING !! ***********************************************************")
                print("\r ******  MAX CURRENT EXEEDED !! *****  Motor:",currents_list[idx], "Value: ",current)
                print(" \r *********************************************************************************")
                print("\n")
                sys.exit()
        total_current += current

        
        
        if debug >= 3: 
            print("\r  Current", idx, current,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
    if debug >= 1:
        print("\r  ============================================================")
        print("\n  ->->->->->-> total_current ->->->->-> ", total_current)  
        print("\r  ============================================================")    

    




def display_ang_torques():
    global motor_torques
    global debug
    
    
    for idx, motor in enumerate(motors):
        #    motor.update
        motors[idx].update()
        
  
        # Initialize an empty dictionary
    torques_list = ["Torque3", "Torque2", "Torque6", "Torque8", "Torque5", "Torque4", "Torque7", "Torque9", "Torque10", "Torque11", "Torque12"]
    torques_dict = {} 
    for idx, motor in enumerate(motors):

        torque = motor.torque
        motor_torques[idx] = torque
        
            #Create a dictionary for publish values
        torques_dict[torques_list[idx]] = round(torque,2)
        
        
        if debug >= 3: 
            print("\r  torque", idx, torque,"Amps ")
            #print("\r  Temperature", idx, motor._motor_state.temperature," C ")  # entrega basura
            
        
    
    if debug >= 1:


            # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", motor_torques)
            # Join the formatted strings into a single string and print
          
        print(f"  motor_torques (New m): {', '.join(formatted_numbers)}")
        print("\r  ============================================================")
        
            #Publish the torques
        topic = "body/monitor/torques"
        print(torques_dict)
            # Serialize dictionary to JSON string with newline after each key-value pair
        json_string = json.dumps(torques_dict, indent=4)  # Pretty print with indentation
        json_string_with_newline = '\n'.join(json_string.split('\n'))

            # Publish JSON string to MQTT broker
        publish_message(mqttClient, topic, json_string_with_newline)
      
def millis():
    return round(time.time() *1000)
    
def init_motors():
    display_ang_positions()
    for idx, motor in enumerate(motors):
        print("\r  Setting zero motor ", idx)    
        motor.set_zero_position()
        time.sleep(1.5) # wait for the motors to zero (~1 second)
    display_ang_positions()  
   
   
   #receives an angular position (rads) and delivers a XYZ position
def direct_kinematics(current_position) :
    #global old_XYZ_position
    
    #adjust angles to respect to pos ini vertical
    
    # convert to vertical refereced

    alpha1 = current_position[0]-current_position[1]
    alpha2 = -current_position[1] #the negative in link 2 (musle) is required for the correct conversion to vertical coordinates
    if debug >= 2:
        print("\r        direct kinematics")
        #print("\r  current_position (rads)", current_position)
            # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(current_position))
            # Join the formatted strings into a single string and print
        print(f"        current_position (degrees): {', '.join(formatted_numbers)}")
        print("\r           alpha1:",  np.degrees(alpha1), "degrees")
        print("\r           alpha2:",  np.degrees(alpha2), "degrees\n") 
#this requires angles referenced to vertical
    V1z = L1 * np.cos(alpha1)
    V1x = L1 * np.sin(alpha1)


    V2z = L2 * np.cos(alpha2)
    V2x = L2 * np.sin(alpha2)

    V3x = V1x + V2x
    V3z = V1z + V2z
    V3y = 0.0

    #old_XYZ_position = [V3x,V3y,V3z]
    ##print("V3x = ",V3x, "V3z =", V3z)
    
    return V3x, V3y, V3z
      
def inverse_kinematics(x, z):
    """
    Calculates the inverse kinematics of a 2-link vertical open chain.

    Parameters:
        x (float): x-coordinate of the end effector.
        z (float): y-coordinate of the end effector.


    Returns:
        theta1 (float): Angle of the first joint in radians.
        theta2 (float): Angle of the second joint in radians.
        
    """
    if debug >= 1: print(" \r Inverse kinematics:  x = ", x, "  z =  ", z)
    
    theta1 = 0
    theta2 = 0
    error = False
    try:
        if  z == 0:
            raise ValueError("some value of ({}, {}) is zero.".format(x, z))
        
        
        
        L = np.sqrt(x**2 + z**2)

        # Check if the desired point is reachable
        if L > L1 + L2 or L < np.abs(L1 - L2):
            raise ValueError("Point ({}, {}) is unreachable.".format(x, z))

        # Calculate theta2
        cos_theta2 = (L**2 - L1**2 - L2**2) / (2 * L1 * L2)
        sin_theta2 = np.sqrt(1 - cos_theta2**2)
        theta2 = np.arctan2(sin_theta2, cos_theta2)

        # Calculate theta1
        beta = np.arccos((L1**2 + L**2 - L2**2) / (2 * L1 * L))
        theta1 = np.arctan2(z, x) - beta
    except ZeroDivisionError:
        print("\n Error: Division by zero!")
        error = True
    except ValueError:
        print("\n Error: Invalid inputs, either zero or too large.")
        error = True
        
    return theta1, theta2, error

def init_gains():
    print("\n Init motor gains")
    for idx, motor in enumerate(motors):
        print("\r  Setting gains motor ", idx) 
        motor.set_impedance_gains_real_unit(K=15.0,B=0.5)
        #time.sleep(1)

# receives the desired angular position (in radians) and Sends them to the motors  
def set_ang_position(positions):
    
    global debug
    global positions_anterior
    
    # Define two np arrays for element-wise compaison
    array1 = np.array(positions)
    array2 = np.array(positions_anterior)

    # Check if array1 is not equal to array2
    if not np.array_equal(array1, array2):

        positions_anterior = positions
        if debug >= 2:
            print("\n **** set_ang_position function ****")
            #print("\n   Desired positions (rads):  ", positions, "rads")    
            # Convert each float to a string with 2 decimal places
            formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(positions))
            # Join the formatted strings into a single string and print
            print(f" Desired positions (degrees): {', '.join(formatted_numbers)}")  
            
        angles = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]    
        for idx, motor in enumerate(motors):
            angles[idx] =  ZERO_POSITIONS[idx]+ positions[idx]
            motor.position =  angles[idx]
            motor.update

#Moves to a XYZ position by using the inverse kinematic model, no trajectory!
def set_XYZ_position(XYZ_position):
    

    new_ang_position, error = get_new_ang_position(XYZ_position)
    if error:
        print("\r Error in inverse kinematic ")
    else: 
        
        set_ang_position(new_ang_position) 
        display_XYZ_positions()            
                    
                    
                    
 #Moves to a XYZ position by using a trajectory of traj_num_steps points ans using the inverse kinematic model  
 #This function eters and leaves moving point by point every time it is called (every Period)  
 #The trajectory begins at the currrent XYZ position, named old_XYZ_position
def trajectory_XYZ(XYZ_position):
    print("\r**** Trajectory ****")
    global old_XYZ_position
    global command
    
    
    display_XYZ_positions()   #For updating old_XYZ_position
    
     # Define the number of steps in the trajectory
    traj_num_steps = 100
    traj_idx= 0
    Period = 10
    Millis_ant = 0
    # Define the start and end points of the trajectory use npumpy arrays for convenience
    start_point = np.array(old_XYZ_position)  # [x, y, z]
    end_point = np.array(XYZ_position)    # [x, y, z]
        # Generate intermediate points along the trajectory using linear interpolation
    trajectory = np.linspace(start_point, end_point, traj_num_steps)

    while traj_idx < len(trajectory):
        
        CurrentMillis = millis()
        if (CurrentMillis - Millis_ant) >= Period:  # Entra cada "Period" segundos" 
            Millis_ant = CurrentMillis
                             
            set_XYZ_position(trajectory[traj_idx])
            traj_idx+=1 
            

    print("Trajectory finished")
    traj_idx= 0
 
    command = "" #This makes sure the trajectory_XYZ function is not called indefinetly
                              
def get_new_ang_position(XYZ_position):
    global INI_POSITIONS
    update_home_positions # why do I have to do this?
    if debug >= 1: print("\r get_new_ang_position")  
    
    x = XYZ_position[0]
    z = XYZ_position[2]
   
   
      # get the  angles (in rads) with respect to horizontal
    theta1, theta2, error = inverse_kinematics(x, z)
    
    if not error:
        #calculate theta2 measured on top of vector1
        theta2=theta2+theta1

        # Angles to respect to vertical 
        Theta1m = theta1
        Theta2m = np.pi-theta2
        #Angles to respect to motors
        Theta1m = np.pi/2-Theta1m
        Theta2m = np.pi/2-Theta2m
    
        if debug >= 1:
            print("\n Angles of inverse model -respect to horizontal-")
            print("Theta1=", np.degrees(theta1), "degrees")
            print("Theta2=", np.degrees(theta2), "degrees")
        
            print("\n Angles to respect to motors")
            print("\r Theta1m =",np.degrees(Theta1m))
            print("\r Theta2m =",np.degrees(Theta2m))


            new_ang_position = INI_POSITIONS
            new_ang_position[0] = Theta1m
            new_ang_position[1] = Theta2m
            new_ang_position[4] = -Theta1m
            new_ang_position[5] = -Theta2m
        else:
            print("Error in inverse kinematic")   
        
        
        return new_ang_position, error
                    
 #Moves to a XYZ position by using a trajectory of traj_num_steps points  using the inverse kinematic model  
 #This function enters and leaves moving point by point every time it is called (every Period) 
  #The trajectory begins at the curent angular position called old_ang_position
def trajectory_ang_XYZ(XYZ_position):
    print("\r**** Trajectory_ang_XYZ initiated ****")
  
    #Get the angular position corresponding to the desired XYZ position
    new_ang_position, error = get_new_ang_position(XYZ_position) #this calls the inverse kinematic model 
    if error:
        print("\r Error in inverse kinematic, abort command! ")
        return  # Quit the function early  
        
    trajectory_ang_space(new_ang_position)
         
   #This function eters and leaves moving point by point every time it is called (every Period)          
def execute_trajectory():
    global old_ang_position
    global desired_ang_position
    global traj_idx
    global trajectory  
    global  trajectory_finished
    global debug
     
    if traj_idx < len(trajectory):
        old_ang_position =   trajectory[traj_idx]         
        set_ang_position(old_ang_position)
        if debug >= 1:
                print("\r ===============================>>>>>>>>>>>>>>>>>>>  Executing trajectory  Traj_idx = ", traj_idx)
        traj_idx+=1 
    else:
        traj_idx= 0
        trajectory = []
        trajectory_finished = True
        desired_ang_position = old_ang_position
        print("\r =====================================================================================================")
        print("\rtrajectory_ang_space finished")    
        print("\r =====================================================================================================")
        
 #Moves to a XYZ position by using a trajectory of traj_num_steps points in angular space. It DOES NOT use the inverse kinematic model  !
  #This function enters and leaves moving point by point every time it is called (every Period) 
 #The trajectory begins at the cuurent angular position called old_ang_position
def trajectory_ang_space(local_new_ang_position):
    print("\r**** trajectory_ang_space initiated ****")
    global old_ang_position
    global command
    global debug
    global traj_idx
    global trajectory
    global  trajectory_finished
    
    #display_XYZ_positions()   #For updating old_ang_position  NO LONGER NECCESARY IN V11
     # Define the number of steps in the trajectory
    traj_num_steps = 100
    traj_idx= 0
    trajectory_finished = False

    # Define the start and end points of the trajectory use npumpy arrays for convenience
    start_point = np.array(old_ang_position)  # [8 angles]
    end_point = np.array(local_new_ang_position)    # [8 angles]
        # Generate intermediate points along the trajectory using linear interpolation
        
    if debug >= 1:
        
        print("\n   start_point (rads):  ", start_point, "rads")    
        # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(start_point))
        # Join the formatted strings into a single string and print
        print(f" start_point (degrees): {', '.join(formatted_numbers)}")  
        
        print("\n   end_point (rads):  ", end_point, "rads")    
        # Convert each float to a string with 2 decimal places
        formatted_numbers = map(lambda x: f"{x:.2f}", np.degrees(end_point))
        # Join the formatted strings into a single string and print
        print(f" end_point (degrees): {', '.join(formatted_numbers)}")   
        
     #Create a trajectory for EACH JOINT!           
    trajectory = np.linspace(start_point, end_point, traj_num_steps)
    command = "" 

def home():
    global INI_POSITIONS
    update_home_positions # why do I have to do this?

    print("\n****************** INI_POSITIONS: ",np.degrees(INI_POSITIONS))
    #set_ang_position(INI_POSITIONS)
    
    trajectory_ang_space(INI_POSITIONS)

def reset():
    global ZERO_POSITIONS
    #set_ang_position(INI_POSITIONS)
    trajectory_ang_space(ZERO_POSITIONS)    
   
   
def setup_up_down():
    global thetas_up_down
    
    print("\r ***** up_down *****")    
          # Generate thetas values
    delta_angle = np.pi/4.0
    resolution = 100
    thetas_up_down = np.linspace(0.0,delta_angle , resolution)  # 100 points from 0 to delta_angle . Importante empezar desde ZERO, para evitar un brinco a la primera posicion!!
   
      
   
# This function enters and leaves every Period, Howeve, it CREATES its own trajector, that is: it does not call  trajectory_ang_space(positions)
#It sends position directly to de motors                             
def up_down():
    global debug
    global num_cycles
    global local_cycles
    global INI_POSITIONS
    global UP_DOWN_enable_joints
    update_home_positions  # why do I have to do this?
    
    global thetas_up_down
    global idx_theta
    global direction
    global desired_ang_position
    global command

    
    
    if local_cycles < num_cycles:
        

             
            theta = thetas_up_down[idx_theta] 
            #print("\r idx_theta",idx_theta)   
            #print ("\r theta",theta)  
            
            new_ang_positions = []
            for idx, angle in enumerate (INI_POSITIONS):
                sign = amplitudes[idx]*UP_DOWN_enable_joints[idx]
                #add theta to every motor angle position in INI_POSITIONS, mask the angles with the enabled joints and  the correct sign
                new_ang_positions.append(angle  + sign*theta) 
            #set_ang_position(new_ang_positions)
            desired_ang_position = new_ang_positions
            
            idx_theta += direction
            if idx_theta >= len(thetas_up_down):
                direction = -1  # Reverse direction
                idx_theta = len(thetas_up_down) - 2  # Start from the second-to-last element
            elif idx_theta < 0:
                direction = 1  # Reverse direction
                idx_theta = 1  # Start from the second element
                local_cycles+=1 
                print("\r local_cycles",local_cycles)      
    else:
        local_cycles = 0
        direction = 1  # Direction of traversal (1 for increasing, -1 for decreasing)
        idx_theta = 0
        command = ""   # VERY IMPORTANT, to finish the movement!


        
def setup_sinusolidal_Jogging_step(cycles):
    global sin_trajectory
    global jog_delay_sin_lower_leg
    global jog_delay_sin_upper_leg
    global jog_delay_sin_anckle
    global jog_delay_sin_neck
    global main_period
    global half_size_step
    global CM_frequency_factor 
    
    global INI_POSITIONS
    update_home_positions  # why do I have to do this?
   
    print("\r ***** setup_sinusolidal_Jogging_step *****")
   
   # Parameters
    sampling_time = 0.010 

    # Define parameters
    #main_period = 2.0   # global variable: this is the period of the first sinusoidal functio
    # Frequency of the first sinusoidal function
    main_frequency = 1 /  main_period

    amplitude1 = 1.0  # amplitude of the first sinusoidal function
    amplitude2 = 1.0  # amplitude of the second sinusoidal function
    amplitude3 = 1.0  # amplitude of the third sinusoidal function
    amplitude4 = 1.0  # amplitude of the third sinusoidal function
    
    #  lower leg sinusoid2 delay in seconds, this is a global variable
    # jog_delay_sin_lower_leg = main_period/16


    # Calculate the frequency of the second sinusoidal function
    if jog_delay_sin_lower_leg == main_period/2:
        frequency2 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency2 =( 1 / (main_period/2 - jog_delay_sin_lower_leg))/2


        # Upper leg sinusoid3 delay in seconds, this is a global variable
    #jog_delay_sin_upper_leg = main_period/8


    # Calculate the frequency of the second sinusoidal function
    if jog_delay_sin_upper_leg == main_period/2:
        frequency3 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency3 =( 1 / (main_period/2 - jog_delay_sin_upper_leg))/2

    # Calculate the frequency of the second sinusoidal function
    if jog_delay_sin_upper_leg == main_period/2:
        frequency4 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency4 =( 1 / (main_period/2 - jog_delay_sin_anckle))/2


    # Time values
    
    #duration = num_cycles* Period  # Duration of the trajectory (in seconds)
    duration = main_period/2  # Duration of the trajectory (in seconds)
    num_samples = int(duration/ sampling_time) # Number of samples
    #num_samples = 1000
    print("num_samples:",num_samples )
    phase = 0.0 # np.pi / 2   # Phase shift (in radians)


    # Time array
    t = np.linspace(0, duration, num_samples)

    # Create sinusoidal data for both functions without delay
    sinusoid11 = amplitude1 * np.sin(2 * np.pi * main_frequency * t*CM_frequency_factor)
    sinusoid21 = amplitude2 * np.sin(2 * np.pi * frequency2 * t)
    sinusoid31 = amplitude3 * np.sin(2 * np.pi * frequency3 * t)
    sinusoid41 = amplitude4 * np.sin(2 * np.pi * frequency4 * t)


    
    # Calculate delay in samples
    delay_samples_sinusoid2 = int(jog_delay_sin_lower_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid21_delayed = np.roll(sinusoid21, delay_samples_sinusoid2)

    # Adjust the delayed sinusoid to start from zero
    sinusoid21_delayed[:delay_samples_sinusoid2] = 0




    
    # Calculate delay in samples
    delay_samples_sinusoid3 = int(jog_delay_sin_upper_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid31_delayed = np.roll(sinusoid31, delay_samples_sinusoid3)

    # Adjust the delayed sinusoid to start from zero
    sinusoid31_delayed[:delay_samples_sinusoid3] = 0

    
    # Calculate delay in samples
    delay_samples_sinusoid4 = int(jog_delay_sin_anckle / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid41_delayed = np.roll(sinusoid41, delay_samples_sinusoid4)

    # Adjust the delayed sinusoid to start from zero
    sinusoid41_delayed[:delay_samples_sinusoid4] = 0



    # **** Concatenate both half_size_step *****

    if half_size_step == 2:
        sinusoid1 = np.concatenate((sinusoid11,-sinusoid11))
        sinusoid2 = np.concatenate((sinusoid21_delayed,-sinusoid21_delayed))
        sinusoid3 = np.concatenate((sinusoid31_delayed,-sinusoid31_delayed))
        sinusoid4 = np.concatenate((sinusoid41_delayed,-sinusoid41_delayed))
    # **** left half step *****

    if half_size_step == 1:
        sinusoid1 = sinusoid11
        sinusoid2 = sinusoid21_delayed
        sinusoid3 = sinusoid31_delayed
        sinusoid4 = sinusoid41_delayed
        
    total_duration = cycles * main_period       # Duration of the trajectory (in seconds)
    total_num_samples = cycles * 2* num_samples   # Number of samples


    #**** Repeat the steps n times ****
    # Repeat the array n times
    sinusoid1 = np.tile(sinusoid1, cycles)
    sinusoid2 = np.tile(sinusoid2, cycles)
    sinusoid3 = np.tile(sinusoid3, cycles)
    sinusoid4 = np.tile(sinusoid4, cycles)

    duration2 = cycles * half_size_step*duration
    num_samples2 = int(cycles * half_size_step*num_samples)
    t2 = np.linspace(0, duration2, num_samples2)

    zeros_trajectory = np.zeros(num_samples2)
    # Stack sinusoidal data into columns of a global array
    sin_trajectory = np.column_stack(( sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid1, sinusoid4, sinusoid4))

    print("sin_trajectory:",sin_trajectory )

    #Plotting the sinusoidal functions from the global array
    plt.figure(figsize=(10, 6))
    plt.plot(t2, sin_trajectory[:, 0], label=f'Sinusoid M3 - {frequency2} Hz')
    plt.plot(t2, sin_trajectory[:, 1], label=f'Sinusoid M2 - {frequency3} Hz')
    plt.plot(t2, sin_trajectory[:, 2], label=f'Sinusoid M6 - {main_frequency} Hz')
    plt.plot(t2, sin_trajectory[:, 3], label=f'Sinusoid M9 - {0} Hz')
    plt.plot(t2, sin_trajectory[:, 4], label=f'Sinusoid M5 - {frequency2} Hz')
    plt.plot(t2, sin_trajectory[:, 5], label=f'Sinusoid M4 - {frequency3} Hz')
    plt.plot(t2, sin_trajectory[:, 6], label=f'Sinusoid M7 - {main_frequency} Hz')
    plt.plot(t2, sin_trajectory[:, 7], label=f'Sinusoid M8 - {0} Hz')
    plt.plot(t2, sin_trajectory[:, 8], label=f'Sinusoid M10 - {main_frequency} Hz')
    plt.plot(t2, sin_trajectory[:, 9], label=f'Sinusoid M12 - {frequency4} Hz')
    plt.plot(t2, sin_trajectory[:, 10], label=f'Sinusoid M11 - {frequency4} Hz')


    plt.title('Four Sinusoidal Functions with delay  Shift')
    plt.xlabel('Time (seconds)')
    plt.ylabel('amplitude')
    plt.legend()
    plt.grid(True)
    plt.show()





    print("sin_trajectory:",sin_trajectory )
    idx_theta = 0     
 

    
    
def setup_sinusolidal_step(cycles):
    global sin_trajectory
    global step_delay_sin_lower_leg
    global step_delay_sin_upper_leg
    global step_delay_sin_anckle
    global step_delay_sin_neck  #Not used yet   
    global main_period
    global half_size_step
    global CM_frequency_factor 
    
    global INI_POSITIONS
    update_home_positions  # why do I have to do this?
   
    print("\r ***** setup_sinusolidal_step *****")
   
   # Parameters
    sampling_time = 0.010 

    # Define parameters
    #main_period = 2.0   # global variable: this is the period of the first sinusoidal functio
    # Frequency of the first sinusoidal function
    main_frequency = 1 /  main_period

    amplitude1 = 1.0  # amplitude of the first sinusoidal function
    amplitude2 = 1.0  # amplitude of the second sinusoidal function
    amplitude3 = 1.0  # amplitude of the third sinusoidal function
    amplitude4 = 1.0  # amplitude of the third sinusoidal function
    
    #  lower leg sinusoid2 delay in seconds, this is a global variable
    # step_delay_sin_lower_leg = main_period/16


    # Calculate the frequency of the second sinusoidal function
    if step_delay_sin_lower_leg == main_period/2:
        frequency2 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency2 =( 1 / (main_period/2 - step_delay_sin_lower_leg))/2


        # Upper leg sinusoid3 delay in seconds, this is a global variable
    #step_delay_sin_upper_leg = main_period/8


    # Calculate the frequency of the second sinusoidal function
    if step_delay_sin_upper_leg == main_period/2:
        frequency3 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency3 =( 1 / (main_period/2 - step_delay_sin_upper_leg))/2

    # Calculate the frequency of the second sinusoidal function
    if step_delay_sin_upper_leg == main_period/2:
        frequency4 = main_frequency *2  # Set a default frequency if delay equals period
    else:
        frequency4 =( 1 / (main_period/2 - step_delay_sin_anckle))/2


    # Time values
    
    #duration = num_cycles* Period  # Duration of the trajectory (in seconds)
    duration = main_period/2  # Duration of the trajectory (in seconds)
    num_samples = int(duration/ sampling_time) # Number of samples
    #num_samples = 1000
    print("num_samples:",num_samples )
    phase = 0.0 # np.pi / 2   # Phase shift (in radians)


    # Time array
    t = np.linspace(0, duration, num_samples)

    # Create sinusoidal data for both functions without delay
    sinusoid11 = amplitude1 * np.sin(2 * np.pi * main_frequency * t*CM_frequency_factor)
    sinusoid21 = amplitude2 * np.sin(2 * np.pi * frequency2 * t)
    sinusoid31 = amplitude3 * np.sin(2 * np.pi * frequency3 * t)
    sinusoid41 = amplitude4 * np.sin(2 * np.pi * frequency4 * t)


    #****** First Half of step ******
    # Calculate delay in samples
    delay_samples_sinusoid2 = int(step_delay_sin_lower_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid21_delayed = np.roll(sinusoid21, delay_samples_sinusoid2)

    # Adjust the delayed sinusoid to start from zero
    sinusoid21_delayed[:delay_samples_sinusoid2] = 0




    #****** Second Half of step ******
    # Calculate delay in samples
    delay_samples_sinusoid3 = int(step_delay_sin_upper_leg / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid31_delayed = np.roll(sinusoid31, delay_samples_sinusoid3)

    # Adjust the delayed sinusoid to start from zero
    sinusoid31_delayed[:delay_samples_sinusoid3] = 0

    #****** Second Half of step ******
    # Calculate delay in samples
    delay_samples_sinusoid4 = int(step_delay_sin_anckle / (t[1] - t[0]))

    # Apply delay to the second sinusoid
    sinusoid41_delayed = np.roll(sinusoid41, delay_samples_sinusoid4)

    # Adjust the delayed sinusoid to start from zero
    sinusoid41_delayed[:delay_samples_sinusoid4] = 0



    # **** Concatenate both half_size_step *****

    if half_size_step == 2:
        sinusoid1 = np.concatenate((sinusoid11,-sinusoid11))
        sinusoid2 = np.concatenate((sinusoid21_delayed,-sinusoid21_delayed))
        sinusoid3 = np.concatenate((sinusoid31_delayed,-sinusoid31_delayed))
        sinusoid4 = np.concatenate((sinusoid41_delayed,-sinusoid41_delayed))
    # **** left half step *****

    if half_size_step == 1:
        sinusoid1 = sinusoid11
        sinusoid2 = sinusoid21_delayed
        sinusoid3 = sinusoid31_delayed
        sinusoid4 = sinusoid41_delayed
        
    total_duration = cycles * main_period       # Duration of the trajectory (in seconds)
    total_num_samples = cycles * 2* num_samples   # Number of samples


    #**** Repeat the steps n times ****
    # Repeat the array n times
    sinusoid1 = np.tile(sinusoid1, cycles)
    sinusoid2 = np.tile(sinusoid2, cycles)
    sinusoid3 = np.tile(sinusoid3, cycles)
    sinusoid4 = np.tile(sinusoid4, cycles)

    duration2 = cycles * half_size_step*duration
    num_samples2 = int(cycles * half_size_step*num_samples)
    t2 = np.linspace(0, duration2, num_samples2)

    zeros_trajectory = np.zeros(num_samples2)
    # Stack sinusoidal data into columns of a global array
    sin_trajectory = np.column_stack(( sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid2, sinusoid3, sinusoid1, zeros_trajectory, sinusoid1, sinusoid4, sinusoid4))

    print("sin_trajectory:",sin_trajectory )

    #Plotting the sinusoidal functions from the global array
    plt.figure(figsize=(10, 6))
    plt.plot(t2, sin_trajectory[:, 0], label=f'Sinusoid M3 - {frequency2} Hz')
    plt.plot(t2, sin_trajectory[:, 1], label=f'Sinusoid M2 - {frequency3} Hz')
    plt.plot(t2, sin_trajectory[:, 2], label=f'Sinusoid M6 - {main_frequency} Hz')
    plt.plot(t2, sin_trajectory[:, 3], label=f'Sinusoid M9 - {0} Hz')
    plt.plot(t2, sin_trajectory[:, 4], label=f'Sinusoid M5 - {frequency2} Hz')
    plt.plot(t2, sin_trajectory[:, 5], label=f'Sinusoid M4 - {frequency3} Hz')
    plt.plot(t2, sin_trajectory[:, 6], label=f'Sinusoid M7 - {main_frequency} Hz')
    plt.plot(t2, sin_trajectory[:, 7], label=f'Sinusoid M8 - {0} Hz')
    plt.plot(t2, sin_trajectory[:, 8], label=f'Sinusoid M10 - {main_frequency} Hz')
    plt.plot(t2, sin_trajectory[:, 9], label=f'Sinusoid M12 - {frequency4} Hz')
    plt.plot(t2, sin_trajectory[:, 10], label=f'Sinusoid M11 - {frequency4} Hz')


    plt.title('Four Sinusoidal Functions with delay  Shift')
    plt.xlabel('Time (seconds)')
    plt.ylabel('amplitude')
    plt.legend()
    plt.grid(True)
    plt.show()





    print("sin_trajectory:",sin_trajectory )
    idx_theta = 0     
 

# This function enters and leaves every Period, Howeve, it CREATES its own trajector, that is: it does not call  trajectory_ang_space(positions)
#It sends position directly to de motors  
def sinusolidal_step(amplitudes):
    global debug
    global num_cycles
    global local_cycles
    global INI_POSITIONS
    update_home_positions  # why do I have to do this?
    
    global sin_trajectory
    global idx_theta
    global direction
    global desired_ang_position
    global command

    
    
    if local_cycles < num_cycles:
        

             
            theta = sin_trajectory[idx_theta] #selects row by row.. Every row has 8 angles corresponding to the 8 motors
            #print("\r idx_theta",idx_theta)   
            #print ("\r theta",theta)  
            
            new_ang_positions = []
            for idx, ini_angle in enumerate (INI_POSITIONS):
                joint_angle_amplitude = amplitudes[idx]
                joint_angle = theta[idx]
                new_ang_positions.append(ini_angle  + joint_angle_amplitude*joint_angle) #add (trajectory angles * amplitudes) to every motor angle position in INI_POSITIONS
            #set_ang_position(new_ang_positions)
            desired_ang_position = new_ang_positions
            
            idx_theta += 1
            if idx_theta >= len(sin_trajectory):
                idx_theta = 0
                local_cycles+=1         

                print("\r local_cycles",local_cycles)      
    else:
        local_cycles = 0
        idx_theta = 0
        command = ""   # VERY IMPORTANT, to finish the movement!
            

 
def jogging_step(amplitudes):
    global debug
    global num_cycles
    global local_cycles
    global INI_POSITIONS
    update_home_positions  # why do I have to do this?
    
    global sin_trajectory
    global idx_theta
    global direction
    global desired_ang_position
    global command

    
    
    if local_cycles < num_cycles:
        

             
            theta = sin_trajectory[idx_theta] #selects row by row.. Every row has 8 angles corresponding to the 8 motors
            #print("\r idx_theta",idx_theta)   
            #print ("\r theta",theta)  
            
            new_ang_positions = []
            for idx, ini_angle in enumerate (INI_POSITIONS):
                joint_angle_amplitude = amplitudes[idx]
                joint_angle = theta[idx]
                new_ang_positions.append(ini_angle  + joint_angle_amplitude*joint_angle) #add (trajectory angles * amplitudes) to every motor angle position in INI_POSITIONS
            #set_ang_position(new_ang_positions)
            desired_ang_position = new_ang_positions
            
            idx_theta += 1
            if idx_theta >= len(sin_trajectory):
                idx_theta = 0
                local_cycles+=1         

                print("\r local_cycles",local_cycles)      
    else:
        local_cycles = 0
        idx_theta = 0
        command = ""   # VERY IMPORTANT, to finish the movement!
            


   
                                     
def step():
    global num_cycles
    global INI_POSITIONS
    update_home_positions # why do I have to do this?
    
     # Generate thetas values
    delta_angle = np.pi/4.0
    resolution = 100
    thetas = np.linspace(-delta_angle,delta_angle , resolution)  # 100 points from 0 to delta_angle . Importante empezar desde ZERO, para evitar un brinco a la primera posicion!!
       
    #first_step(delta_angle)
    
    
    

    #np_positions = np.array(INI_POSITIONS)
    
   
    direction = 1  # Direction of traversal (1 for increasing, -1 for decreasing)
    idx_theta = 0
    Millis_ant = 0
    Period = 100 
    local_cycles = 0
    
    while local_cycles < num_cycles:
        
        CurrentMillis = millis()
        if (CurrentMillis - Millis_ant) >= Period:  # Entra cada "Period" segundos" 
            Millis_ant = CurrentMillis
                               
            theta = thetas[idx_theta] 
            
            new_ang_positions = []
            for idx, angle in enumerate (INI_POSITIONS):
                sign = amplitudes[idx]
                new_ang_positions.append(angle  + abs(sign)*theta) #add theta to every motor angle position in INI_POSITIONS
            set_ang_position(new_ang_positions)
            
            idx_theta += direction
            if idx_theta >= len(thetas):
                direction = -1  # Reverse direction
                idx_theta = len(thetas) - 2  # Start from the second-to-last element
            elif idx_theta < 0:
                direction = 1  # Reverse direction
                idx_theta = 1  # Start from the second element
                local_cycles+=1 

    local_cycles = 0
    debug = 1
    display_XYZ_positions()
 
def condini():
    global INI_POSITIONS
    global desired_ang_position
    
    setup_up_down()
    setup_sinusolidal_step(1) #cycles must be integer
    
    
    desired_ang_position = ZERO_POSITIONS  
    
    print("\n****************** INI_POSITIONS: ",np.degrees(INI_POSITIONS))
    
                                     
def robot_control():
    global command
    global new_ang_position
    global old_ang_position
    global old_XYZ_position
    global desired_ang_position
    global  trajectory_finished
    global salir
    global debug
    
    global step_upper_leg_ang
    global step_lower_leg_ang 
    global step_CM_displacement_ang 
    global main_period
    global step_delay_sin_lower_leg 
    global step_delay_sin_upper_leg 
    global half_size_step 
    global step_turn_ang 
    global STEP_amplitudes
    
    print("Starting control. Press ctrl+C to quit.")
   
   

    condini()
                
                
    Millis_ant = 0
    Period = 10
     
    count = 0        
    salir = False
    
    trajectory_finished = True
    read_variables()#this updates old_ang_position & old_XYZ_position
    publish_variables()  
    
    while  not(salir):
  
        CurrentMillis = millis()
        if (CurrentMillis - Millis_ant) >= Period:  # Entra cada "Period" segundos" 
            Millis_ant = CurrentMillis
            
                #every 10 mSeg
            read_variables()  #this updates old_ang_position & old_XYZ_position
            if not trajectory_finished:
                execute_trajectory()
            else:    
                set_ang_position(desired_ang_position)
                
            count+=1
                #every 100 mSeg
            if count >= 10:
                count = 0
                publish_variables()
                display_RPY_angles()   
                
             
            if command == "home":
                print("\r Going home")
                debug = 1
                home()
                command ="" 
            if command == "reset":
                print("\r Going reset")
                debug = 1
                reset()
                command ="" 
                
            if command == "up_down":
                #print("\r Going up_down")
                debug = 1
                #home() #send it to home first to avoid a jump to the middle position
                up_down()#uses the global variable num_cycles
                
                #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles
                
            if command == "Forward_step":
                #print("\r Going step")
                debug = 1
                #home() #send it to home first to avoid a jump to the middle position
                #step()#uses the global variable num_cycles
                    

                # = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang]
                #STEP_amplitudes = np.radians(STEP_amplitudes)
                sinusolidal_step(STEP_amplitudes)
                #home() #send it to home last to return to the middle position
                #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles
                
            if command == "Jogging_step":
                #print("\r Going step")
                debug = 1
                #home() #send it to home first to avoid a jump to the middle position
                #step()#uses the global variable num_cycles
                    

                # = [step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang,step_lower_leg_ang,-step_upper_leg_ang,step_CM_displacement_ang,step_turn_ang]
                #STEP_amplitudes = np.radians(STEP_amplitudes)
                sinusolidal_step(Jogging_amplitudes)
                #home() #send it to home last to return to the middle position
                #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles  
                
            if command == "side_step":
                #print("\r Going side_step")
                debug = 1
                #home() #send it to home first to avoid a jump to the middle position
                #step()#uses the global variable num_cycles
                sinusolidal_step(SIDE_STEP_amplitudes)
                #home() #send it to home last to return to the middle position
                #command =""  DO NOT pu this line. command needs to be unchangend to let the program enters here until the end of the cycles                
                
            elif command == "positionXYZ":
                debug = 1
                set_XYZ_position(new_XYZ_position)
                command =""
            elif command == "TrajectoryXYZ":
                debug = 1
                #trajectory_XYZ(new_XYZ_position)
                trajectory_ang_XYZ(new_XYZ_position)
                command =""   
            elif command == "monitor_angles":
                debug = 2
                display_ang_positions()
                command =""
            elif command == "monitor_XYZ":
                debug = 1
                display_XYZ_positions()
                command =""
            elif command == "monitor_RPY":
                debug = 1
                display_RPY_angles()
                command =""
            elif command == "monitor_torques":
                debug = 1
                display_ang_torques()  
                command =""                
            elif command == "monitor_currents":
                debug = 1
                display_ang_currents()   
                command =""     

     

        
            

 #####################################################################
  #  main program 
 ####################################################################    


if __name__ == '__main__':
    

# to use additional motors, simply add another with block
# remember to give each motor a different log name!
    print("\r baby Rowdy control program  Sergio Montufar UTSA")
    print("\r Spring 2024 Version: ", VERSION)
                                                      
    print("\r +++++++++++++++ Initialization:  ++++++++++++++  ")   
    
##########################################################
    # MQTT setup

    print("Setting Up MQTT")

    # Flag to indicate subscribe confirmation hasn't been printed yet.
    didPrintSubscribeMessage = False

    print('----------------------------------------')

    mqttClient = mqtt.Client()

    # Set up calling functions to mqttClient

    mqttClient.on_connect = on_connect  # attach function to callback
    mqttClient.on_message = on_message  # attach function to callback

    # Connect to the MQTT server  in the local LAN & loop forever.
    # CTRL-C will stop the program from running.
    print("server address is:", serverAddress)
    mqttClient.connect(serverAddress)

    print('----------------------------------------')
 
    #mqttClient.loop_forever()# use this line if you don't want to write any further code. It blocks the code forever to check for data

    # Start the MQTT client in a non-blocking thread
    mqttClient.loop_start() #use this line if you want to write any more code here to execute along the mqtt client

    # Write your main program here:
    # Main program continues to run concurrently

  
    
    
    
 
                            
    motor_list = ["Theta3", "Theta2", "Theta6", "Theta8", "Theta5", "Theta4", "Theta7", "Theta9", "Theta10", "Theta11", "Theta12"] 
      
    with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_1) as dev1:
        with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_2) as dev2:
            with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_3) as dev3:
                with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_4) as dev4:
                        with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_5) as dev5:
                            with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_6) as dev6:
                                with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_7) as dev7:
                                    with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_8) as dev8:
                                         with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_9) as dev9:
                                             with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_10) as dev10:
                                                with TMotorManager_mit_can(motor_type=Type_1, motor_ID=ID_11) as dev11: 
                                                    #Add motor objects to a list called motors
                                                    motors.extend([dev1,dev2,dev3,dev4,dev5,dev6,dev7,dev8,dev9,dev10,dev11])
                                                    motors_dict = dictionary = dict(zip(motor_list, motors))
                                                    print(motors_dict)


                                                    debug=1
                                                    init_motors()
                                                    
                                                    init_gains()
                                                    
                                                    display_ang_positions()
                                                    #home()  #The supports put it in home
                                                    #print("Going home")
                                                    #set_ang_position(INI_POSITIONS)                                      
                                                    robot_control()
                                                    if salir :
                                                            sys.exit()
                                            
            
     #except:
