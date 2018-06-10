#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
	
from geometry_msgs.msg import Twist

import keyboard
import pyautogui as pag
import time

msg = """
w-forward
a-left starfing
s-backward
d-right starfing

+ speed 10% up
- speed 10% down

"""
fpstime = 0.02

moveBindings = {
		'w':(1,0),
		'a':(0,1),
		's':(-1,0),
		'd':(0,-1),
	       }

def inc_limit(inc, value, max_lim):
	if value>0:
		if value < max_lim:
			if inc < 0:
				value += 8*inc
			else:
				value += inc
		else:
			value = max_lim
	else :
		if value > -max_lim:
			if inc > 0:
				value += 8*inc
			else:
				value += inc
	return value

def dec_limit(dec, value):
	if value > dec:
		value -= dec
	elif value< -dec:
		value += dec
	elif -dec<value and value < dec:
		value = 0
	return value

speed_inc = 0.15
dec = 0.5

speed_max = 7.5



width,height = pag.size()
yaw_dpi = 2
pag.moveTo(width/2,height/2)
pitch = 0
yaw = 0
pi = 3.1416

if __name__=="__main__":	
	pub = rospy.Publisher('cmd_vel_1', Twist, queue_size = 0)
	rospy.init_node('teleop_twist_keyboard_1')

	x = 0
	y = 0


	status = 0 
	x_speed = 0
	y_speed = 0
	turn = 0


	try:
		print msg
		kb = keyboard.KBHit()
		while(1):
			mx,my = pag.position()
			pitch = - (my-height/2)*pi/height
			yaw = yaw_dpi*(mx-width/2)*pi/2/width
			
			pag.moveTo(width/2,my)
			if kb.kbhit():
				key = kb.getch()
			else:
				key = ' '
			
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				x_speed=inc_limit(x*speed_inc,x_speed,speed_max)
				y_speed=inc_limit(y*speed_inc,y_speed,speed_max)

				
			else:
				x_speed = dec_limit(dec,x_speed)
				y_speed = dec_limit(dec,y_speed)
				if (key == 'q'):
					break
			
		
			twist = Twist()
			twist.linear.x = x_speed; twist.linear.y = y_speed; twist.linear.z = 0;
			twist.angular.x = 0; twist.angular.y = pitch; twist.angular.z = yaw
			pub.publish(twist)
			print twist.linear.x ,twist.linear.y , twist.angular.y , twist.angular.z 
			time.sleep(fpstime)

	except KeyboardInterrupt:
		print 'end..............'

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)



