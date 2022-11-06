from dis import dis
import cv2
import numpy as np 
from dronekit import connect, VehicleMode ,Vehicle , LocationLocal , LocationGlobal, LocationGlobalRelative

import time
from pymavlink import mavutil

vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
try:
    def arm_and_takeoff(aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print ("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not vehicle.is_armable:
            print (" Waiting for vehicle to initialise...")
            time.sleep(1)

        print ("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode    = VehicleMode("GUIDED")
        vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print (" Waiting for arming...")
            time.sleep(1)

        print ("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print (" Altitude: ", vehicle.location.global_relative_frame.alt)
            #Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print ("Reached target altitude")
                break
            time.sleep(1)

except KeyboardInterrupt:
    vehicle.mode = VehicleMode("LAND")
    vehicle.armed= False


arm_and_takeoff(1.5)




#sleep to make sure to reach the right spot

#defining a function to go move in NEP with given velocity

try:
    def send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration=0):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
        for x in range(0,duration):
            vehicle.send_mavlink(msg)
            time.sleep(1)
except KeyboardInterrupt:
    vehicle.mode = VehicleMode("LAND")
    vehicle.armed= False
    



# turning on the camera and staring image processing 

cap = cv2.VideoCapture(0)

_, prev = cap.read()
prev = cv2.flip(prev, 1)
_, new = cap.read()
new = cv2.flip(new, 1)

_, frame = cap.read()
frame =cv2.flip(frame, 1)
blurred_frame = cv2.GaussianBlur(frame, (101,101), 0)

hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    
lower_blue = np.array([38, 86, 0])
upper_blue = np.array([121, 255, 255])
mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

while True:
    diff = cv2.absdiff(prev, new)
    diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    diff = cv2.blur(diff, (5,5))
 
    _,thresh = cv2.threshold(diff, 10, 255, cv2.THRESH_BINARY)
    threh = cv2.dilate(thresh, None, 3)  # type: ignore
    thresh = cv2.erode(thresh, np.ones((4,4)), 1)  # type: ignore
 
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    height, width = mask.shape[:2]
    
 
 
    distance_x =0
    distance_y =0
    
    if len(contours)!=0:
 
        max_contour =contours[0]
        max_contour_area =cv2.contourArea(max_contour)
    
        for contors in contours:				
            if cv2.contourArea(contors) > max_contour_area:
                max_contour = contors
                max_contour_area = cv2.contourArea(max_contour)
                
    
        cv2.circle(prev, (320,240), 5, (0,255,0), -1)
        contors =  max_contour			
        if cv2.contourArea(contors) > 300:
            (x,y,w,h) = cv2.boundingRect(contors)
            (x1,y1),rad = cv2.minEnclosingCircle(contors)

            x1 = int(x1)
            y1 = int(y1)

            distance_x = x1-320
            distance_y = y1-240

            if (int(np.sqrt((distance_x)**2 + (distance_y)**2)) > 100):
       
                    cv2.line(prev, (320,240), (x1, y1), (255,0,0), 4)
                    cv2.putText(prev, "{}".format(int(np.sqrt((distance_x)**2 + (distance_y)**2))), (100,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 3)
                    cv2.rectangle(prev, (x,y), (x+w,y+h), (0,255,0), 2)
                    cv2.circle(prev, (x1,y1), 5, (0,0,255), -1)
                    if distance_y >  0:
                        send_body_ned_velocity(0.5,(distance_y/distance_x),0)
                    elif distance_y < 0 :
                        send_body_ned_velocity(-0.5,(distance_y/distance_x),0)
                    else:
                        send_body_ned_velocity(0,0.7,0)
                        
                    
                    
                
                    
    cv2.imshow("orig", prev)
    cv2.imshow("Mask", mask)
        
    prev = new
    _, new = cap.read()
    new = cv2.flip(new, 1)
    blurred_frame=cv2.GaussianBlur(new, (101,101), 0)
    hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([38, 86, 0])
    upper_blue = np.array([121, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv2.contourArea(contour) > 1 :
            cv2.drawContours(frame, contour, -1, (0, 255, 0), 1)

    
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()

vehicle.mode = VehicleMode("LAND")
vehicle.armed= False
