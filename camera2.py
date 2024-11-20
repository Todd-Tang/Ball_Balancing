import cv2
import numpy as np
from time import sleep

# '***' stands for things to modify for your own webcam, display, and ball if needed

# Define a function to detect a yellow ball
def detect_yellow_ball():
    # Start capturing video from the webcam. If multiple webcams connected, you may use 1,2, etc.
    cap = cv2.VideoCapture(0)
    # *1 CAP_PROP_FPS sets the frame rate of the webcam to 30 fps here
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # *2 Set the image resolution to 480x480. Note increasing resolution increases processing power used, and may slow down video feed.
        frame = cv2.resize(frame, (720, 720))

        # Convert the frame from BGR to HSV color space to easily identify a colour
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

        # *3 Define the range of yellow color in HSV [Hue, Saturation, Value]
        # SET THESE VALUES VIA THE METHOD EXPLAINED IN THE TUTORIAL
        """
        ball_color_lower = np.array([20, 100, 100]) # [lower Hue, lower Saturation, lower Value]
        ball_color_upper = np.array([30, 255, 255]) # [upper Hue, upper Saturation, upper Value]
        """

        ball_color_lower = np.array([0, 4, 112]) # [lower Hue, lower Saturation, lower Value]
        ball_color_upper = np.array([179, 76, 255])

        # Threshold the HSV image to get the colors defined above
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
        kernel = np.ones((4, 4), np.uint8) 
        #mask = cv2.inRange(hsv, ball_color_lower, ball_color_upper)
        mask = cv2.Canny(hsv, 100, 150)
        #mask = cv2.dilate(mask, kernel, iterations=1)
        # mask = cv2.erode(mask, kernel, iterations=1)


        # Find contours in the mask
        # RETR_TREE retrieves all hierarchical contours and organizes them
        # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
        
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            # Determines the larget contour size using the cv2.contour Area function
            largest_contour = max(contours, key=cv2.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            # * 4 Only consider large enough objects. If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
            if radius > 10:
                # Draw a yellow circle around the ball
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
                cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                # Display the position of the ball
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
        """

        image = cv2.bitwise_not(mask)
        
        # Set our filtering parameters 
        # Initialize parameter setting using cv2.SimpleBlobDetector 
        # params = cv2.SimpleBlobDetector_Params() 

        # params.minThreshold = 10
        # params.maxThreshold = 200
        
        # # Set Area filtering parameters 
        # params.filterByArea = True
        # params.minArea = 500
        # params.maxArea = 1000000
        
        # # Set Circularity filtering parameters 
        # params.filterByCircularity = True 
        # params.minCircularity = 0.1
        # params.maxCircularity = 1
        
        # # Set Convexity filtering parameters 
        # params.filterByConvexity = True
        # params.minConvexity = 0.5
        # params.maxConvexity = 1
            
        # # Set inertia filtering parameters 
        # params.filterByInertia = True
        # params.minInertiaRatio = 0.001
        # params.maxInertiaRatio = 1
        
        # Create a detector with the parameters 
        # detector = cv2.SimpleBlobDetector_create(params) 

        #mask = cv2.cvtColor(mask, cv2.HSV2GRAY)

        detected_circles = cv2.HoughCircles(mask,  
                        cv2.HOUGH_GRADIENT, 1, 20, param1 = 50, 
                    param2 = 30, minRadius = 1, maxRadius = 40) 
        
        # Draw circles that are detected. 
        if detected_circles is not None: 
        
            # Convert the circle parameters a, b and r to integers. 
            detected_circles = np.uint16(np.around(detected_circles)) 
        
            for pt in detected_circles[0, :]: 
                a, b, r = pt[0], pt[1], pt[2] 
        
                # Draw the circumference of the circle. 
                cv2.circle(mask, (a, b), r, (0, 255, 0), 2) 
        
                # Draw a small circle (of radius 1) to show the center. 
                cv2.circle(mask, (a, b), 1, (0, 0, 255), 3) 
        else:
            detected_circles = []

        number_of_blobs = len(detected_circles) 
        text = "Number of Circular Blobs: " + str(len(detected_circles)) 
        cv2.putText(number_of_blobs, text, (20, 550), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2) 
        
        # Show blobs 
        cv2.imshow("Filtering Circular Blobs Only", mask) 

        # Display the resulting frame
        #cv2.imshow('frame', frame)
        #cv2.imshow('mask', mask)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv2.destroyAllWindows()

# Call the function to detect the yellow ball
detect_yellow_ball()
