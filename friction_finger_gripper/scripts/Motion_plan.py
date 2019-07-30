#!/usr/bin/env python

import numpy as np

import rospy
import time
from friction_finger_gripper.srv import*
# from visualservoing.srv import *

from geometry_msgs.msg import Point
from common_msgs_gl.srv import *
from std_msgs.msg       import Float32
from std_msgs.msg       import Int32


FINGER_END = 15
FINGER_START = 0
SLIDING_RESOLUTION = 0.1
ROTATION_SLIDE_RESOLUTION=0.2
PALM_WIDTH = 5
OBJECT_SIZE= 2.5
TH1_MAX= 2.485 #142.5 degrees
TH2_MIN= 0.65 #37.5
FINGER_WIDTH=1
Block_orientation=0
Block_Position=[0,0]
VISUAL_SERVOING_ENABLE=0


def angle_conversion(angle, flag):
    angle = 180. * angle / np.pi
    # 0-> Left, 1-> Right
    print 'angle = ', angle
    if(flag == 1):
        n_angle =  0.002244*angle+ 0.563388
    else:
        n_angle = -0.002223*angle + 1.043235
    print("n_angle = ", n_angle)
    return (n_angle)

def finger_to_cartesian(L,R,A,th):
    if A=="r_plus" or A=="r_minus":
        x_square = (L - OBJECT_SIZE/2.0)*np.cos(np.float64(th[0])) + (FINGER_WIDTH + OBJECT_SIZE/2.0)*np.sin(np.float64(th[0]))
        # x_square = (R - (OBJECT_SIZE/2.0))
        y_square = (L - OBJECT_SIZE/2.0)*np.sin(np.float64(th[0])) - (FINGER_WIDTH + OBJECT_SIZE/2.0)*np.cos(np.float64(th[0]))


    elif A=="l_plus" or A=="l_minus":
        x_square = PALM_WIDTH + (R - OBJECT_SIZE/2.0)* np.cos(th[1]) - (OBJECT_SIZE/2.0 + FINGER_WIDTH)* np.sin(th[1])
        y_square = (R - OBJECT_SIZE/2.0)* np.sin(th[1]) + (OBJECT_SIZE/2.0 + FINGER_WIDTH)* np.cos(th[1])


    return x_square,y_square



def encoder_gripper_angle_conversion(enc,flag):
    # if(flag==1):
    #     theta=-(enc-1.03396)/0.0029
    # else:
    #     theta=(enc-0.14021)/0.00389
    # return theta
    if(flag==1):
        theta=(enc-0.563388)/0.002244
    else:
        theta=-(enc-1.043235)/0.002223
    return theta


def slide_left_finger_down(p):
    # Call Left_Slide_Down(till left most position) Assume t1 = pi/6
    rospy.wait_for_service('Slide_Left_Finger_Down')
    try:
        slide_left_down = rospy.ServiceProxy('Slide_Left_Finger_Down', PositionCommand)
        resp1 = slide_left_down(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_left_finger_up(p):
    rospy.wait_for_service('Slide_Left_Finger_Up')
    try:
        slide_left_up = rospy.ServiceProxy('Slide_Left_Finger_Up', PositionCommand)
        resp1 = slide_left_up(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_right_finger_down(p):
    # Call Left_Slide_Down(till left most position) Assume t1 = pi/6
    rospy.wait_for_service('Slide_Right_Finger_Down')
    try:
        slide_right_down = rospy.ServiceProxy('Slide_Right_Finger_Down', PositionCommand)
        resp1 = slide_right_down(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)


def slide_right_finger_up(p):
    rospy.wait_for_service('Slide_Right_Finger_Up')
    try:
        slide_right_up = rospy.ServiceProxy('Slide_Right_Finger_Up', PositionCommand)
        resp1 = slide_right_up(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)

def rotate_object_anticlockwise(p):
    rospy.wait_for_service('Rotate_anticlockwise')
    try:
        rotate_anti = rospy.ServiceProxy('Rotate_anticlockwise',PositionCommand)
        resp1 = rotate_anti(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)

def rotate_object_clockwise(p): #0.35
    rospy.wait_for_service('Rotate_clockwise')
    try:
        Rotate_clockwise = rospy.ServiceProxy('Rotate_clockwise',PositionCommand)
        resp1 = Rotate_clockwise(p)
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)


def read_pos():
    rospy.wait_for_service('read_pos')
    try:
        read_position_handler = rospy.ServiceProxy('read_pos', GetDoubleArray)
        values = read_position_handler()
        print values.data
        return values.data
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)



def Correction_step(goal_x,goal_y):
    rospy.wait_for_service('Visual_servoing')
    try:
        vis_servo_client=rospy.ServiceProxy('Visual_servoing',Visual_servo_goal)
        resp=vis_servo_client(goal_x,goal_y)
        return resp
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)



def orientation_callback(msg):
     global Block_orientation
     
     #Radians to degree conversion(Values lies between -180 to 180)
     if(msg.data>=0 and msg.data<=3.14):
        Block_orientation=msg.data*180/np.pi
     else:
        Block_orientation=180+ abs(msg.data)*180/np.pi
        
     #print Block_orientation

def position_callback(msg):
     global Block_Position
     
     Block_Position=Point(msg.x*100,msg.y*100,msg.z*100)

def Call_visual_servo(goal_x,goal_y):

    
        print "Position correction"
        print "Goalx,goaly=",goal_x,goal_y

        if(Correction_step(goal_x,goal_y)):
            print "Action position correcion successfully done"

        else:
            print "Action position correction failed"


if __name__ == '__main__':
    rospy.init_node('Control_loop')
    list_of_lists=[]
    #with open('p.txt') as f:
    with open('/home/gsathyanarayanan/finger_ws/src/Motion_planner/data1.txt') as f:    
        for line in f:
    	   inner_list = [elt.strip() for elt in line.split(',')]
    	   list_of_lists.append(inner_list)

    action_list=[]
    theta1=[]
    theta2=[]
    x_position=[]
    y_position=[]
    for row in list_of_lists:
    	action_list.append(float(row[2]))
    	theta1.append(float(row[3]))
    	theta2.append(float(row[4]))
        x_position.append(float(row[5]))
        y_position.append(float(row[6]))

    smooth_actions=[]
    a= action_list[0]   
    n_actions= len(action_list)
    for i in range(n_actions):
        if i==0:
            continue
        if(action_list[i]==a):
            continue
        else:
            smooth_actions.append((action_list[i-1],theta1[i-1],theta2[i-1],x_position[i-1],y_position[i-1]))
            a=action_list[i]

    smooth_actions.append((action_list[n_actions-1],theta1[n_actions-1],theta2[n_actions-1],x_position[i-1],y_position[i-1]))
    rospy.Subscriber("/object_orientation", Float32, orientation_callback)
    rospy.Subscriber("/object_position",Point, position_callback)
    pub = rospy.Publisher('Action', Int32, queue_size=10000)
    rospy.sleep(1)  # This helps to advertise the topic properly so as all the message can be subscribed
    for i in range(len(smooth_actions)):
        rospy.loginfo(smooth_actions[i])
        
        
        if smooth_actions[i][0]==0:
            pub.publish(int(smooth_actions[i][0]))
            slide_left_finger_down(angle_conversion(smooth_actions[i][1],0))
        elif smooth_actions[i][0]==1:
            pub.publish(int(smooth_actions[i][0]))
            slide_right_finger_down(angle_conversion(smooth_actions[i][2],1))
        elif smooth_actions[i][0]==2:
            pub.publish(int(smooth_actions[i][0]))
            slide_left_finger_up(angle_conversion(smooth_actions[i][2],1))
        elif smooth_actions[i][0]==3:
            pub.publish(int(smooth_actions[i][0]))
            slide_right_finger_up(angle_conversion(smooth_actions[i][1],0))
        elif smooth_actions[i][0]==4:
            if(VISUAL_SERVOING_ENABLE):
                pub.publish(6)
                goal_x,goal_y=smooth_actions[i-1][3],smooth_actions[i-1][4]
                Call_visual_servo(goal_x,goal_y)

            theta=read_pos()
            Motor_value=theta[1]-0.02
            pub.publish(int(smooth_actions[i][0]))
            rotate_object_anticlockwise(Motor_value)
            while(1):
                global Block_orientation
                theta=read_pos()
                
               
                Motor_value=theta[1]-0.02
                #print "Motor_value",Motor_value
                rotate_object_anticlockwise(Motor_value)
                finger_angle=encoder_gripper_angle_conversion(theta[1],1)
                print "Block_orientation=",Block_orientation
                print "Finger_angle=",finger_angle
                print "Diff",(abs(Block_orientation-finger_angle))%90
                condition1=(abs(Block_orientation-finger_angle))%90<7
                condition2=((abs(Block_orientation-finger_angle)))%90>83
                condition3=(abs(Block_orientation+finger_angle))%90<7
                condition4= (abs(Block_orientation+finger_angle))%90>83
                condition5=Block_orientation<=180
                condition6=Block_orientation>=180
                Right_Limit_condition=Motor_value>=0.7
                print "c1=",condition1
                print "c2=",condition2
                print "c3=",condition3
                print "c4=",condition4
                print "c5=",condition5
                print "c6=",condition6
                if ((condition1 or condition2) and condition6 and Right_Limit_condition):
                    break
                if ((condition3 or condition4) and condition5 and Right_Limit_condition):
                    break


        elif smooth_actions[i][0]==5:
            if(VISUAL_SERVOING_ENABLE):
                pub.publish(int(6))
                goal_x,goal_y=smooth_actions[i-1][3],smooth_actions[i-1][4]
                Call_visual_servo(goal_x,goal_y)
            theta=read_pos()
            Motor_value=theta[0]-0.02
                #print "Motor_value",Motor_value
            pub.publish(int(smooth_actions[i][0]))
            rotate_object_clockwise(Motor_value)
            while(1):
                global Block_orientation
                theta=read_pos()
                
               
                Motor_value=theta[0]-0.02
                #print "Motor_value",Motor_value
                rotate_object_clockwise(Motor_value)
                finger_angle=encoder_gripper_angle_conversion(theta[1],1)
                print "Block_orientation=",Block_orientation
                print "Finger_angle=",finger_angle
                print "Diff",(abs(Block_orientation-finger_angle))%90
                condition1=(abs(Block_orientation-finger_angle))%90<7
                condition2=((abs(Block_orientation-finger_angle)))%90>83
                condition3=(abs(Block_orientation+finger_angle))%90<7
                condition4= (abs(Block_orientation+finger_angle))%90>83
                condition5=Block_orientation<=180
                condition6=Block_orientation>=180
                Left_Limit_condition=Motor_value>=0.7
                print "c1=",condition1
                print "c2=",condition2
                print "c3=",condition3
                print "c4=",condition4
                print "c5=",condition5
                print "c6=",condition6
                if ((condition1 or condition2) and condition6 and Left_Limit_condition):
                    break
                if ((condition3 or condition4) and condition5 and Left_Limit_condition):
                    break



        # if (smooth_actions[i][0]==0 or smooth_actions[i][0]==1 or smooth_actions[i][0]==2):
        #     goal_x,goal_y=smooth_actions[i][3],smooth_actions[i][4]

        # elif(smooth_actions[i][0]==3 or smooth_actions[i][0]==4):
        #     goal_x,goal_y=smooth_actions[i+1][3],smooth_actions[i+1][4]

        # if(Correction_step(goal_x,goal_y)):
        #     print "Action position correcion successfully done"

        # else:
        #     print "Action position correction failed"

    if(VISUAL_SERVOING_ENABLE):
        pub.publish(int(6))
        goal_x,goal_y=smooth_actions[len(smooth_actions)-1][3],smooth_actions[len(smooth_actions)-1][4]
        Call_visual_servo(goal_x,goal_y)
    
    print "Motion_Plan_execution_done"

    

    # for action in smooth_actions:
    #     rospy.loginfo(action)
    #     goal_x,goal_y=action[3],action[4]
    #     if action[0]==0:
    #         slide_left_finger_down(angle_conversion(action[1],0))
    #     elif action[0]==1:
    #         slide_right_finger_down(angle_conversion(action[2],1))
    #     elif action[0]==2:
    #         slide_left_finger_up(angle_conversion(action[2],1))
    #     elif action[0]==3:
    #         slide_right_finger_up(angle_conversion(action[1],0))
    #     elif action[0]==4:
    #         rotate_object_anticlockwise(angle_conversion(action[2],1))
    #     elif action[0]==5:
    #         rotate_object_clockwise(angle_conversion(action[1],0))

    #     if(Correction_step(goal_x,goal_y)):
    #         print "Action position correcion successfully done"

    #     else:
    #         print "Action position correction failed"

    # print "Motion_Plan_execution_done"
    # goal_x,goal_y=2.5,10

    # if(Correction_step(goal_x,goal_y)):
    #     print "Goal successfully Reached"

    # else:
    #     print "Correction step failed"


    rospy.spin()

