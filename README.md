# Import libraries
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
GPIO.setwarnings(False)
import YB_Pcb_Car 
car=YB_Pcb_Car.YB_Pcb_Car()

# Webcamera no 0 is used to capture the frames
cap = cv2.VideoCapture(0)
t1=time.time()
# Based on hardware interface manual
EchoPin = 18
TrigPin = 16
AvoidSensorLeft = 21  #Left infrared obstacle avoidance sensor pin
AvoidSensorRight = 19    #Right infrared obstacle avoidance sensor pin
Avoid_ON = 22   #Infrared obstacle avoidance sensor switch pin
GPIO.setmode(GPIO.BOARD)
GPIO.setup(AvoidSensorLeft,GPIO.IN)
GPIO.setup(AvoidSensorRight,GPIO.IN)
GPIO.setup(Avoid_ON,GPIO.OUT)
GPIO.output(Avoid_ON,GPIO.LOW)#Set the GPIO port to BIARD encoding mode

GPIO.setup(EchoPin,GPIO.IN)
GPIO.setup(TrigPin,GPIO.OUT)
def get_distance():
# set Trigger to HIGH
    GPIO.output(TrigPin, GPIO.HIGH)
# set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TrigPin, GPIO.LOW)
    StartTime = time.time()
    topTime = time.time()
# save StartTime
    while GPIO.input(EchoPin) == 0:
        StartTime = time.time()
# save time of arrival
    while GPIO.input(EchoPin) == 1:
          StopTime = time.time()
# Inside the get_distance function
    TimeElapsed = StopTime - StartTime
# sound speed (34300 cm/s)
# and divide by 2
    distance = (TimeElapsed * 34300) / 2
    return distance 
def greenn():
	car.Car_Stop()
	time.sleep(0.002)
	car.Car_Back(100,100)
	time.sleep(0.2)
	car.Car_Left(0,100)
	time.sleep(0.5)
	car.Car_Run(100,100)
	time.sleep(0.5)
	car.Car_Right(100,0)
	time.sleep(0.5)

def redd():
	car.Car_Stop()
	time.sleep(0.002)
	car.Car_Back(100,100)
	time.sleep(0.2)
	car.Car_Right(100,0)
	time.sleep(0.5)
	car.Car_Run(100,100)
	time.sleep(0.5)
	car.Car_Left(0,100)
	time.sleep(0.5)
	



# camera settings
car.Ctrl_Servo(1,140)
car.Ctrl_Servo(2,140)
time.sleep(0.5)


    
# using distance and ir avoid sensor values to control the way of robot movment
def Avoid():
    d=get_distance()
    LeftSensorValue  = GPIO.input(AvoidSensorLeft);
    RightSensorValue = GPIO.input(AvoidSensorRight);

    print(d)
    print(LeftSensorValue )
    print( RightSensorValue)
   

    if d < 45 and LeftSensorValue == 1  and RightSensorValue==0:
            car.Car_Back(80,80)
            time.sleep(0.2)
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Right(80,0) 
            time.sleep(1)
            car.Car_Run(80,80) 
            time.sleep(0.01)
    elif d< 45 and LeftSensorValue== 0  and RightSensorValue==1 :
            car.Car_Back(80,80)
            time.sleep(0.2)
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Left(0,80) 
            time.sleep(1) 
            car.Car_Run(80,80) 
            time.sleep(0.01)
    elif d< 45 and LeftSensorValue== 0  and RightSensorValue==0 :
            car.Car_Back(80,80)
            time.sleep(0.2)
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Left(0,80) 
            time.sleep(.5)   
    elif d< 45 and LeftSensorValue== 1 or RightSensorValue==1 :
            car.Car_Back(80,80)
            time.sleep(0.1)
            car.Car_Stop()
            time.sleep(0.1)
            car.Car_Right(80,0) 
            time.sleep(.5)    
    else:
            car.Car_Run(80,80) 
            time.sleep(0.1)
           
            t2=time.time()
            t3=t2-t1
            print("t2",t2)
            if t3==180:
                car.Car_Stop()
                print("stop")
                time.sleep(10)

 # number of orange lines
 c=0
# main loop used to detect red,green and orange color , and call avoid function infintly until the camera detect 12 orange lines(3 cycles)

while True:
    	
	# Captures the live stream frame-by-frame
    _, frame = cap.read()
	# Converts images from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5,50,50])
    upper_orange= np.array([15,255,255])
    lower_green = np.array([40,100,100])
    upper_green = np.array([80,255,255])
    lower_red = np.array([155,25,0])    
    upper_red = np.array([179,255,255])
	    # Here we are defining range of bluecolor in HSV
	    # This creates a mask of blue coloured
	    # objects found in the frame.
    mask_2= cv2.inRange(hsv, lower_orange , upper_orange)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    red_pixels=np.count_nonzero(mask2)
    green_pixels=np.count_nonzero(mask)
    orange_pixels=np.count_nonzero(mask_2)
   
	    # The bitwise and of the frame and mask is done so
	    # that only the blue coloured objects are highlighted
	    # and stored in res
    res_2 = cv2.bitwise_and(frame,frame, mask= mask_2)
    #cv2.imshow('frame',frame)
    cv2.imshow('mask',mask_2)
    #cv2.imshow('res',res_2)
    # avoid green and red obstecls 
    d2=get_distance()
    if d2<15 and red_pixels> green_pixels:
            redd()
            car.Car_Stop()
            time.sleep(.09)
    elif d2<15 and green_pixels >red_pixels:
            greenn()
            car.Car_Stop()
            time.sleep(.09)
# if the car cross 12 orange lines it will stop
    if orange_pixels>1:
         c+=1
         if c==12:
            break
    Avoid()

    


	    # This displays the frame, mask
	    # and res which we created in 3 separate windows.
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
	    break

# Destroys all of the HighGUI windows.
cv2.destroyAllWindows()

# release the captured frame
cap.release()
GPIO.cleanup()
car.Car_Stop()
time.sleep(10)

    
