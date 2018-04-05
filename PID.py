from ev3dev.ev3 import *
from time import sleep

def colorSum(colorSensor):
	color = colorSensor.value(0) + colorSensor.value(1) + colorSensor.value(2)
	return color	

#min - black, max - white
def calibration():
  left_motor.run_direct(duty_cycle_sp=30)
  right_motor.run_direct(duty_cycle_sp=30)
  max_ref = 0
  min_ref = 1000
  end_time = time() + 5
  while time() < end_time:
    read = (colorSum(cl_right) + colorSum(cl_left))/2
    if max_ref < read:
      max_ref = read
    if min_ref > read:
      min_ref = read
  left_motor.stop()
  right_motor.stop()
  print 'Max: ' + str(max_ref)
  print 'Min: ' + str(min_ref)
	return min_ref, max_ref
  sleep(1)


#Steering module alt. 2: "less aggressive". A less aggressive algorithm that adjusts the power on each motor from 0-100% (never reverse)
def steering(course, power):
	if colorSum(cl_right) < colorSum(cl_left): #if course >= 0: 
		if course > 100:
			power_right = 0
			power_left = power
		else:	
			power_left = power
			power_right = power - ((power * course) / 100)
	else:
		if  course > 100: #course < -100:
			power_left = 0
			power_right = power
		else:
			power_right = power
			power_left = power + ((power * course) / 100)
	return (int(power_left), int(power_right))


def run(power, target, kp, kd, ki, minRef, maxRef):
	lastError = error = integral = 0
	left_motor.run_forever(speed_sp = power)
	right_motor.run_forever(speed_sp = power)
	while(true): #not btn.any() :
		if ts.value():
			print 'Breaking loop' # User pressed touch sensor
			break
		refRead = (colorSum(cl_right) + colorSum(cl_left))/2
		error = target - (100 * ( refRead - minRef ) / ( maxRef - minRef ))
		derivative = error - lastError
		lastError = error
		integral = float(0.5) * integral + error
		course = kp * error + kd * derivative +ki * integral
		power_left, power_right = steering(course, power)
		left_motor.run_forever(speed_sp = power_left)
		right_motor.run_forever(speed_sp = power_right)	
		sleep(0.01)



#Engine Initialization

mB = MediumMotor('outB')
left_motor = LargeMotor('outC');  #assert left_motor.connected
right_motor = LargeMotor('outD'); #assert right_motor.connected

stopSpeed = 0
pickedUp = False

#Sensor Initialization

	#Light
cl_right = ColorSensor('in1')
cl_left = ColorSensor('in2')

cl_right.mode='RGB-RAW'
cl_left.mode='RGB-RAW'

	#Proximity
ifs = InfraredSensor('in3')
	
ifs.mode = 'IR-PROX'
	#Touch
ts = TouchSensor('in4')

#btn = Button()

#VALUES

power = 150
#lub przypisać na stałe
#minRef = 40
#maxRef = 100

#bialy
target = 200
kp = float(0.65)
kd = 1
ki = float(0.02)


min_ref, max_ref = calibration()
run(power, target, kp, kd, ki, minRef, maxRef)

print 'Stopping motors'
left_motor.run_forever(speed_sp = stopSpeed)
right_motor.run_forever(speed_sp = stopSpeed)
