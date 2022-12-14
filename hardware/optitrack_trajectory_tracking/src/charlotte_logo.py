#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 12:10:55 2022

"""
from abs_int import trajectoryPlanning
from control import controller
from control import dynamics
#from IPython.display import display
import numpy as np
import matplotlib.pyplot as plt
import pygame
import pygame.camera 
import rospy
import math
import os
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

rospy.init_node('velocity_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
move = Twist() 
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

global pos_zb, pos_zc

def currentPos1(msg):
        global pos_zb
        pos_zb = msg.pose.position

def currentPos2(msg):
        global pos_zc
        pos_zc = msg.pose.position
        
rospy.Subscriber("/vrpn_client_node/zb/pose", PoseStamped, currentPos1)
rospy.Subscriber("/vrpn_client_node/zc/pose", PoseStamped, currentPos2)
ititial_pose = rospy.wait_for_message("/vrpn_client_node/zb/pose", PoseStamped, rospy.Duration.from_sec(10))
ititial_pose = rospy.wait_for_message("/vrpn_client_node/zc/pose", PoseStamped, rospy.Duration.from_sec(10))

file = open(os.path.join(__location__,'logo.csv'))
position_old = np.loadtxt(file,delimiter=",")# creates positional array from input file'

pt1 = position_old/1000*1.5
pt1[1,:] = pt1[1,:] - 1.5/2*.96
pt1[0,:] = pt1[0,:] - 1.5/2*1.1

base_zbx = pos_zb.x; base_zby = pos_zb.y
base_zcx = pos_zc.x; base_zcy = pos_zc.y

estimated_distance = np.sqrt((base_zbx-base_zcx)**2+(base_zby-base_zcy)**2)
# In[1]:
# D = .04524375
D = estimated_distance
k = 2
m = .1
# poster size 35.5, 45.5 in
h = .03
speed = .05
interp_resolution = .001

v_max = .22
w_max = 2.84


head_int = math.atan2(base_zby-base_zcy,base_zbx-base_zby)

robot_position = [pos_zb.x, pos_zb.y, head_int]
Fullscreen = False
travel_distance_x = 1.5
travel_distance_y = 1.5
# In[2]:
pygame.init()
pygame.camera.init()
infoObject = pygame.display.Info()
white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)
red = (255, 0, 0)
black = (0,0,0)

if Fullscreen== True:
	screen_x = 1022.222222222
	screen_y = 800

else:
	screen_y = 800
	screen_x = 800

def refresh_screen(screen):
    screen.fill(white)
    pygame.draw.line(screen, red, (screen_x/4, 5), (3*screen_x/4, 5), width = 4)
    font = pygame.font.Font('freesansbold.ttf', 15)
    text = font.render("{} m".format(travel_distance_x/2), True, red)
    textRect = text.get_rect()
    textRect.center = (screen_x/2, 19)
    screen.blit(text, textRect)

    pygame.draw.line(screen, red, (5, screen_y/4), (5, 3*screen_y/4), width = 4)
    text = font.render("{} m".format(travel_distance_y/2), True, red)
    textRect.midleft = (19, screen_y/2)
    screen.blit(text, textRect)

def plot_ref(pt):
    lines = []
    T = len(pt)
    for i in range(0,T-1):
        last_position = (round(pt[i,0]*screen_x/travel_distance_x+screen_x/2), round(-pt[i,1]*screen_y/travel_distance_y+screen_y/2))
        lines.append(last_position)
    pygame.draw.lines(screen, blue, False, lines, 3)

def plot_robot(p1, p2):
    pygame.draw.circle(screen, (255,0,0), p1, 5, width=0)
    pygame.draw.circle(screen, (255,255,0), p2, D*4*screen_y/travel_distance_y, width=2)

def plot_velocity(v_max, w_max):
    font = pygame.font.Font('freesansbold.ttf', 25)
    text = font.render("v = {}".format(round(v_max, 2)) + " [m/s]", True, red)
    textRect = text.get_rect()
    textRect.center = (screen_x/2 + screen_x/8, screen_y - 25)
    screen.blit(text, textRect)

    text = font.render("w = {}".format(round(w_max,2)) + " [rad/s]", True, red)
    textRect.center = (screen_x/2 - screen_x/8, screen_y - 25)
    screen.blit(text, textRect)

def plot_time(T):
    font = pygame.font.Font('freesansbold.ttf', 25)
    text = font.render("Time = {}".format(round(T, 2)) + " [s]", True, red)
    textRect = text.get_rect()
    textRect.center = (screen_x/2, 14*screen_y/15)
    screen.blit(text, textRect)    

screen = pygame.display.set_mode((screen_x, screen_y))
pygame.display.set_caption('Trajectory Planning')    
pygame.display.flip()
p1 = (robot_position[0]*screen_x/travel_distance_x+screen_x/2, (-robot_position[1]*screen_y/travel_distance_y+screen_y/2))
p2 = ((robot_position[0]-4*D*np.cos(head_int))*screen_x/travel_distance_x+screen_x/2, (-(robot_position[1]-4*D*np.sin(head_int))*screen_y/travel_distance_y+screen_y/2))

refresh_screen(screen)
plot_ref(np.transpose(pt1))
plot_robot(p1, p2)
pygame.display.update()

position_x = []
position_y = []
lines = []
clock = pygame.time.Clock()
rate = rospy.Rate(1/h)
working = True
path = 0
while (working == True):
    quitting = pygame.key.get_pressed()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            working = False

        elif quitting[pygame.K_ESCAPE]:
            pygame.quit()
            working = False
    
        elif quitting[pygame.K_SPACE]:
            position_x = pt1[0,:]
            position_y = pt1[1,:]

            position_in = np.transpose(np.array([position_x, position_y]))
            waiting_vel = True
            while (waiting_vel == True):
                x_vel = []
                y_vel = []
                v_r = []
                #position = position_in
                position = trajectoryPlanning(position_in, speed, interp_resolution, h)

                T = len(position) 
                for t in range(0,T-1):
                    x_vel.append((position[t+1,0] - position[t,0])/h) # finds velocity from position array on x plane
                    y_vel.append((position[t+1,1]- position[t,1])/h)
                    v_r.append(np.sqrt(x_vel[t]**2+y_vel[t]**2))
                
                if  not v_r: 
                    speed = speed - .01
                    waiting_vel == False
                else:
                    v_max1 = max(v_r) + k*min(m, np.sqrt(robot_position[0]-position[0,0]**2+robot_position[1]-position[0,1]**2))
                    w_max1 = max(v_r)/D + k/D*min(m, np.sqrt(robot_position[0]-position[0,0]**2+robot_position[1]-position[0,1]**2))

                    screen.fill(white)
                    refresh_screen(screen)
                    plot_time(T*h)
                    plot_robot(p1, p2)
                    plot_velocity(v_max1, w_max1)
                    plot_ref(position_in)
                    pygame.display.update()

                    wait_key = True
                    while (wait_key == True):
                        keypress = pygame.key.get_pressed()
                        for event in pygame.event.get():
                            if keypress[pygame.K_SPACE]:
                                waiting_vel = False
                                wait_key = False
                            elif keypress[pygame.K_UP]:
                                if v_max1 < v_max: speed = speed + .01
                                wait_key = False
                            elif keypress[pygame.K_DOWN]:
                                if speed > .01: speed = speed - .01
                                wait_key = False
                            elif keypress[pygame.QUIT]:
                                pygame.quit()
                                working = False
                                wait_key = False
                                waiting_vel = False
                            elif keypress[pygame.K_ESCAPE]:
                                pygame.quit()
                                working = False
                                wait_key = False
                                waiting_vel = False
                    clock.tick(60) 
            clock.tick(60) 
            
            if T > 1 and working == True:
                screen.fill(white)
                refresh_screen(screen)
                plot_robot(p1, p2)
                plot_ref(position_in)
                pygame.display.update()

                xb = np.zeros((T))
                yb = np.zeros((T))
                theta = np.zeros((T))
                state = np.zeros((3,T))
                base_point_lines = []

                xb[0] = pos_zb.x
                yb[0] = pos_zb.y
                theta[0] = robot_position[2]
                state[0,0] = xb[0] - D*np.cos(theta[0])
                state[1,0] = yb[0] - D*np.sin(theta[0])
                state[2,0] = theta[0]
                
                base_lines = (round(xb[0]*screen_x/travel_distance_x+screen_x/2), round(-yb[0]*screen_y/travel_distance_y+screen_y/2))
                base_point_lines.append(base_lines)

                for t in range(0,T-1):
                    x = np.zeros((3,1))

                    x[0,0] = xb[t]
                    x[1,0] = yb[t]
                    x[2,0] = theta[t]

                    v, w = controller(x, x_vel[t], y_vel[t], position[t,0], position[t,1], m, k, D)

                    if v > v_max:
                        v = v_max
                    if v < -v_max:
                        v = -v_max
                    if w > w_max:
                        w = w_max
                    if w < -w_max:
                        w = -w_max
            
                    move.linear.x = v
                    move.angular.z = w
                    pub.publish(move)

                    print(move)
                    rate.sleep()
                    
                    xb[t+1] = pos_zb.x
                    yb[t+1] = pos_zb.y

                    xbc = pos_zc.x
                    ybc = pos_zc.y

                    xdot = xb[t+1]-xb[t]
                    ydot = yb[t+1]-yb[t]
                    
                    theta[t+1] = math.atan2(yb[t+1] -ybc,xb[t+1] -xbc)

                    state[0,t+1] = xb[t+1] - D*np.cos(theta[t+1])
                    state[1,t+1] = yb[t+1] - D*np.sin(theta[t+1])
                    state[2,t+1] = theta[t+1]

                    p1 = (xb[t+1]*screen_x/travel_distance_x+screen_x/2, (-yb[t+1]*screen_y/travel_distance_y+screen_y/2))
                    p2 = ((state[0,t+1]-3*D*np.cos(theta[t+1]))*screen_x/travel_distance_x+screen_x/2, (-(state[1,t+1]-3*D*np.sin(theta[t+1]))*screen_y/travel_distance_y+screen_y/2))

                    base_lines = (round(xb[t+1]*screen_x/travel_distance_x+screen_x/2), round(-yb[t+1]*screen_y/travel_distance_y+screen_y/2))
                    base_point_lines.append(base_lines)

                    screen.fill(white)
                    refresh_screen(screen)
                    plot_robot(p1, p2)
                    pygame.draw.lines(screen, red, False, base_point_lines, 2)
                    plot_ref(position)
                    pygame.display.update()
                move.linear.x = 0
                move.angular.z = 0
                pub.publish(move)
                rospy.is_shutdown() 

                screen.fill(white)
                refresh_screen(screen)
                plot_robot(p1, p2)
                pygame.draw.lines(screen, red, False, base_point_lines, 2)
                pygame.display.update()

                robot_position = [xb[t+1], yb[t+1], theta[t+1]]

                position_x = []
                position_y = []

                base_point = np.zeros((3,len(xb)))
                base_point[0,:] = xb
                base_point[1,:] = yb
                base_point[2,:] = theta
                print("saving...")
                np.savetxt(os.path.join(__location__,"base_point.csv"), base_point, delimiter = ',') 
    if working == True:
        pygame.display.update()
        clock.tick(60)  # Limit the frame rate to 60 FPS.