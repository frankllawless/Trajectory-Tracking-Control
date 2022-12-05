#!/usr/bin/env python
# -*- coding: utf-8 -*-
from abs_int import trajectoryPlanning
from control import controller
from control import dynamics
import numpy as np
import matplotlib.pyplot as plt
import pygame
import pygame.camera 
import rospy
import os
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

rospy.init_node('velocity_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
move = Twist() 
# In[1]:
D = .047625
k = 1
m = .1

h  = .2
speed = .07
interp_resolution = .001

v_max = .22
w_max = 2.84

head_int = 0
robot_position = [D*2,.25, head_int]

Fullscreen = False
travel_distance_x = 1
travel_distance_y = 1
# In[2]:
pygame.init()
pygame.camera.init()
infoObject = pygame.display.Info()
white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)
red = (255, 0, 0)
black = (0, 0, 0)

if Fullscreen== True:
	screen_x = 1400
	screen_y = 700

else:
	screen_x = 1000
	screen_y = 1000

def refresh_screen(screen):
    pygame.draw.line(screen, red, (screen_x/4, 5), (3*screen_x/4, 5), width = 4)
    font = pygame.font.Font('freesansbold.ttf', 15)
    text = font.render("{} m".format(travel_distance_x/2), True, red)
    textRect = text.get_rect()
    textRect.center = (screen_x/2, 19)
    screen.blit(text, textRect)

    pygame.draw.line(screen, red, (5, screen_y/4), (5, 3*screen_y/4), width = 4)
    text = font.render("{} m".format(travel_distance_y/2), True, red)
    textRect.midleft = (19, screen_y/3)
    screen.blit(text, textRect)
    # screen.blit(image, (0, 0))

def plot_robot(p1, p2):
    pygame.draw.circle(screen, (255,255,0), p2, D*screen_y/travel_distance_y+D*screen_y/travel_distance_y, width=5)
    pygame.draw.circle(screen, (255,0,0), p1, 5, width=0)

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
p1 = (robot_position[0]*screen_x/travel_distance_x, (-robot_position[1]*screen_y/travel_distance_y+screen_y/2))
p2 = ((robot_position[0]-D)*screen_x/travel_distance_x-np.cos(robot_position[2])*D*screen_x/travel_distance_x, (-robot_position[1]*screen_y/travel_distance_y+screen_y/2+np.sin(robot_position[2])*D*screen_y/travel_distance_y))
screen.fill(white)
plot_robot(p1, p2)
refresh_screen(screen)

position_x = []
position_y = []
lines = []
clock = pygame.time.Clock()
rate = rospy.Rate(1/h)
working = True
while (working == True):
    quitting = pygame.key.get_pressed()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            working = False

        elif quitting[pygame.K_ESCAPE]:
            pygame.quit()
            working = False

        elif event.type == pygame.MOUSEMOTION:
            if event.buttons[0]:  # Left mouse button down.
                last_position = (event.pos[0] - event.rel[0], event.pos[1] - event.rel[1])
                lines.append(last_position)
                position_x.append(last_position[0]/screen_x*travel_distance_x)
                position_y.append((-last_position[1]+screen_y/2)/screen_y*travel_distance_y)
                pygame.draw.line(screen, black, last_position, event.pos, 2)

        elif quitting[pygame.K_SPACE] and len(position_x) > 0:
            position_in = np.transpose(np.array([position_x, position_y]))
            waiting_vel = True
            while (waiting_vel == True):
                x_vel = []
                y_vel = []
                v_r = []
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
                    pygame.draw.lines(screen, black, False, lines, 1)
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
                    clock.tick(150) 
            clock.tick(150) 
            
            if T > 1 and working == True:
                xb = np.zeros((T))
                yb = np.zeros((T))
                theta = np.zeros((T))
                state = np.zeros((3,T))

                xb[0] = robot_position[0]
                yb[0] = robot_position[1]
                theta[0] = robot_position[2]
                state[0,0] = xb[0] - D*np.cos(theta[0])
                state[1,0] = yb[0] - D*np.sin(theta[0])
                state[2,0] = theta[0]
                
                lines = []
                for i in range(0,T-1):
                    last_position = (round(position[i,0]*screen_x/travel_distance_x), round(-position[i,1]*screen_y/travel_distance_y+screen_y/2))
                    lines.append(last_position)

                pygame.draw.lines(screen, red, False, lines, 2)
                pygame.display.update()

                base_point_lines = []
                base_lines = (round(xb[0]*screen_x/travel_distance_x), round(-yb[0]*screen_y/travel_distance_y+screen_y/2))
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

                    dot_x = dynamics(x,v,w, D)
                    xb[t+1] = xb[t] + dot_x[0]*h
                    yb[t+1] = yb[t] + dot_x[1]*h
                    theta[t+1] = theta[t] + dot_x[2]*h
                    
                    state[0,t+1] = xb[t+1] - D*np.cos(theta[t+1])
                    state[1,t+1] = yb[t+1] - D*np.sin(theta[t+1])
                    state[2,t+1] = theta[t+1]
        
                    move.linear.x = v 
                    move.angular.z = w
                    pub.publish(move)

                    print(move)
                    rate.sleep()

                    base_lines = (round(xb[t+1]*screen_x/travel_distance_x), round(-yb[t+1]*screen_y/travel_distance_y+screen_y/2))
                    base_point_lines.append(base_lines)

                    p1 = (xb[t+1]*screen_x/travel_distance_x, (-yb[t+1]*screen_y/travel_distance_y+screen_y/2))
                    p2 = (state[0,t+1]*screen_x/travel_distance_x-np.cos(theta[t+1])*D*screen_x/travel_distance_x, (-state[1,t+1]*screen_y/travel_distance_y+screen_y/2+np.sin(theta[t+1])*D*screen_y/travel_distance_y))

                    screen.fill(white)
                    refresh_screen(screen)
                    plot_robot(p1, p2)
                    pygame.draw.lines(screen, (255,0,0), False, base_point_lines, 2)
                    pygame.display.update()
                move.linear.x = 0
                move.angular.z = 0
                pub.publish(move)
                rospy.is_shutdown() 
                
                robot_position = [xb[t+1], yb[t+1], theta[t+1]]

                position_x = []
                position_y = []
                lines = []

    if working == True:
        pygame.display.update()
        clock.tick(60)  # Limit the frame rate to 60 FPS.