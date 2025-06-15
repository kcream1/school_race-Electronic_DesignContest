import sensor, image, time
import car
from pid import PID
from pyb import Pin,LED
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(10)
sensor.set_auto_whitebal(False)
KEY4=Pin('P4',Pin.IN,Pin.PULL_UP)
KEY5=Pin('P5',Pin.IN,Pin.PULL_UP)
KEY6=Pin('P6',Pin.IN,Pin.PULL_UP)
KEY9=Pin('P9',Pin.IN,Pin.PULL_UP)
a=0
b=0
clock = time.clock()
size_threshold = 15800
x_pid = PID(p=0.1, i=0.3, imax=10)
h_pid = PID(p=0.0018, i=0.0007, imax=10)
def find_max(blobs):
	max_size=0
	for blob in blobs:
		if blob[2]*blob[3] > max_size:
			max_blob=blob
			max_size = blob[2]*blob[3]
	return max_blob
while(True):
	clock.tick()
	img = sensor.snapshot()


	if KEY9.value() == 0:
		orange = (100, 34, 10, 114, 11, 118)
		blobs = img.find_blobs([orange])
		if KEY4.value() == 0:
			while(~KEY4.value()):
				LED(1).on()
				LED(2).on()
				LED(3).on()
				if KEY4.value()==1:
					break
			a=a+1
			LED(1).off()
			LED(2).off()
			LED(3).off()
		if a>=4:
			a=0
		if a==1:
			LED(1).off()
			LED(2).off()
			LED(3).off()
			LED(1).on()
			red = (33, 56, 18, 53, -53, 16)
			blobs = img.find_blobs([red])
		if a==2:
			LED(1).off()
			LED(2).off()
			LED(3).off()
			LED(2).on()
			lv = (19, 97, -69, -34, -29, 127)
			blobs = img.find_blobs([lv])
		if a==3:
			LED(1).off()
			LED(2).off()
			LED(3).off()
			LED(3).on()
			blue=(62, 43, 7, -18, -24, -91)
			blobs = img.find_blobs([blue])


		if KEY5.value() == 0:
			while(~KEY5.value()):
				LED(1).on()
				LED(2).on()
				LED(3).on()
				if KEY5.value()==1:
					break
			LED(1).off()
			LED(2).off()
			LED(3).off()
			b=b+1
		if b>=4:
			b=0
		if b==1:
			LED(1).off()
			LED(2).off()
			LED(3).off()
			LED(1).on()
			LED(2).on()
			huang = (49, 98, -54, 39, 11, 27)
			blobs = img.find_blobs([huang])
		if b==2:
			LED(1).off()
			LED(2).off()
			LED(3).off()
			LED(1).on()
			LED(3).on()
			zi = (84, 50, 21, 4, -30, -12)
			blobs = img.find_blobs([zi])
		if b==3:
			LED(1).off()
			LED(2).off()
			LED(3).off()
			LED(2).on()
			LED(3).on()
			qing=(73, 76, -4, 9, 18, -15)
			blobs = img.find_blobs([qing])
	else:
		LED(1).off()
		LED(2).off()
		LED(3).off()
		orange = (100, 34, 10, 114, 11, 118)
		huang = (49, 98, -54, 39, 11, 27)
		lv = (19, 97, -69, -34, -29, 127)
		zi = (84, 50, 21, 4, -30, -12)
		red = (33, 56, 18, 53, -53, 16)
		blue=(62, 43, 7, -18, -24, -91)
		blobs = img.find_blobs([red,zi,blue,lv,orange,huang],merge=True)
	if blobs:
		max_blob = find_max(blobs)
		x_error = max_blob[5]-img.width()/2
		h_error = max_blob[2]*max_blob[3]-size_threshold
		print("x error: ", x_error)
		img.draw_rectangle(max_blob[0:4])
		img.draw_cross(max_blob[5], max_blob[6])
		x_output=x_pid.get_pid(x_error,1)
		h_output=h_pid.get_pid(h_error,1)
		print("h_output",h_output)
		car.run(-h_output-x_output,-h_output+x_output)
	else:
		car.run(10,-10)