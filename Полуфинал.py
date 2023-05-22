import pymurapi as mur
import time
import cv2
import numpy as np

mur.mur_init()
auv = mur.mur_init()

wait = time.sleep

motor_gepth_l = 2
motor_gepth_r = 3
motor_turn_l = 0
motor_turn_r = 1

h = 0
ignore, ignore1, exitt = True, True, 0

prev_time = 0.0
prev_error_d = 0.0

def PD(error, Kp, Kd, max_power, prev_error):
    global prev_time
    current_time = int(round(time.time() * 1000))
    diff_value = Kd / (current_time - prev_time) * (error - prev_error)
    u = error * Kp + diff_value
    u = max_power if u > max_power else u
    u = -max_power if u < -max_power else u
    return u


def set_depth(depth_target=2.8):
    global prev_error_d, prev_time
    error = auv.get_depth() - depth_target
    kp = 200
    kd = 200000
    max_power = 30
    u = PD(error, kp, kd, max_power,prev_error_d)
    motors_depth(u)
    prev_time, prev_error_d = int(round(time.time() * 1000)), error
    wait(0.2)
    
def motors_depth(power_l, power_r=None):
    power_r = power_r or power_l
    auv.set_motor_power(motor_gepth_l, power_l)
    auv.set_motor_power(motor_gepth_r, power_r)
    
    
def motors_turn(power_l, power_r=None):
    power_r = power_r or power_l
    auv.set_motor_power(motor_turn_l, power_l)
    auv.set_motor_power(motor_turn_r, power_r)
    
    
def get_cnt_xy(contour):
    moments = cv2.moments(contour)
    try:
        x = int(moments['m10'] / moments['m00'])
        y = int(moments['m01'] / moments['m00'])
        return x,y
    except ZeroDivisionError:
        return None, None

def stab_on_some_circle(cnt, damage):
    global ignore
    global ignore1
    global h
    global exitt
    for c in cnt:
        x, y = get_cnt_xy(c)
    if x:
        errorX = 160 - x
        errorY = 120 - y
        if -20 <= errorX <= 20 and -20 <= errorY <=20:
            auv.drop()
            h += damage
            exitt += 1
            ignore = False
            ignore1 = True
            auv.set_motor_power(motor_turn_l, 0)
            auv.set_motor_power(motor_turn_r, 0)
            wait(2)
        else:
            ignore1 = False
            if errorY < 0:
                auv.set_motor_power(motor_turn_l, errorY)
                auv.set_motor_power(motor_turn_r, errorY)
            else:
                if errorX > 0:
                    auv.set_motor_power(motor_turn_r, (abs(errorX) * 3 + errorY) * 0.1)
                    auv.set_motor_power(motor_turn_l, errorY * 0.1)
                elif errorX < 0:
                    auv.set_motor_power(motor_turn_l, (abs(errorX) * 3 + errorY) * 0.1)
                    auv.set_motor_power(motor_turn_r, errorY * 0.1)
        

def yellow_or_green(cnt, damage):
    if cnt:
        for c in cnt:
            area = cv2.contourArea(c)
            if abs(area) < 300:
                continue
            else:
                return cnt, damage
    return None, None
    
    
def find_circles(image):
    global ignore
    global ignore1
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    green_hsv_mask = cv2.inRange(imageHSV, (55,50,50), (80,255,255))
    yellow_hsv_mask = cv2.inRange(imageHSV, (20,50,50), (40,255,255))
    cnt, __ = cv2.findContours(green_hsv_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt1, _ = cv2.findContours(yellow_hsv_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt, damage = yellow_or_green(cnt, 1)
    if not cnt:
        cnt, damage = yellow_or_green(cnt1, 2)
    if cnt and ignore:
        stab_on_some_circle(cnt, damage)
    elif not cnt:
        ignore1 = True
        ignore = True

def move_over_trumpet(image):
    image = image[:20,:]
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    purple_hsv_color = cv2.inRange(imageHSV, (125,43,46),(135,255,255))
    cnt, _ = cv2.findContours(purple_hsv_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnt:
        x = 0
    for c in cnt:
        moments = cv2.moments(c)
        try:
            x = int(moments['m10'] / moments['m00'])
        except ZeroDivisionError:
            x = 0
    errorX = x - 160
    errorX = 45 if errorX > 45 else errorX
    errorX = -15 if errorX < -15 else errorX
    if x > 0:
        motors_turn(20, 20 - errorX)
    if x < 0:
        motors_turn(20 - errorX, 20)

def isExit():
    if exitt == 5:
        if h % 2:
            while True:
                auv.set_motor_power(motor_turn_r, 100)
                auv.set_motor_power(motor_turn_l, -100)
                auv.set_motor_power(motor_gepth_l,100)
                auv.set_motor_power(motor_gepth_r, 100)
                
        else:
            while True:
                auv.set_motor_power(motor_turn_r, -100)
                auv.set_motor_power(motor_turn_l, 100)
                auv.set_motor_power(motor_gepth_l, 100)
                auv.set_motor_power(motor_gepth_r, 100)
while True:
    isExit()
    set_depth()
    image = auv.get_image_bottom()
    if auv.get_depth() > 2.5:
        find_circles(image)
        if ignore1:
            move_over_trumpet(image)
                
                
