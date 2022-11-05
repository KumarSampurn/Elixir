from dis import dis
import cv2
import numpy as np 

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
	# cv2.circle(prev, (20,200), 5, (0,255,0), -1) 	height, width = img.shape[:2]
 
 
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
