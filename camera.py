import cv2
import numpy as np
from time import sleep
import time

# '***' stands for things to modify for your own webcam, display, and ball if needed
ball_location = None
last_location = None
resolution_scaling_factor = 3
resolution = (int(1280/resolution_scaling_factor), int(720/resolution_scaling_factor))
plate_centre = [int((522) / (resolution_scaling_factor)),int((720 - 350)/resolution_scaling_factor)]
camera_rotation = -4

is_running = True

# Define a function to detect a yellow ball
def detect_ball():
    global ball_location, last_location
    # Start capturing video from the webcam. If multiple webcams connected, you may use 1,2, etc.

    
    # Read a frame from the webcam
    cap = cv2.VideoCapture(0)
    # *1 CAP_PROP_FPS sets the frame rate of the webcam to 30 fps here
    cap.set(cv2.CAP_PROP_FPS, 30)

    fps = 0
    dps = 0
    last_time = 0
    while is_running:

        sleep(0.0) # yield control
        fps += 1
        if (time.time() - last_time) > 1:
            print(f"FPS: {fps}, DPS: {dps}")
            fps = 0
            dps = 0
            last_time = time.time()

        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # *2 Set the image resolution to 480x480. Note increasing resolution increases processing power used, and may slow down video feed.
        frame = cv2.resize(frame, resolution)

        #M = cv2.getRotationMatrix2D(plate_centre, -camera_rotation, 1.0)
        #frame = cv2.warpAffine(frame, M, (resolution[0], resolution[1]))

        # Convert the frame from BGR to HSV color space to easily identify a colour
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

        # *3 Define the range of yellow color in HSV [Hue, Saturation, Value]
        # SET THESE VALUES VIA THE METHOD EXPLAINED IN THE TUTORIAL
        """
        ball_color_lower = np.array([20, 100, 100]) # [lower Hue, lower Saturation, lower Value]
        ball_color_upper = np.array([30, 255, 255]) # [upper Hue, upper Saturation, upper Value]
        """

        ball_color_lower = np.array([8, 80, 100]) # [lower Hue, lower Saturation, lower Value]
        ball_color_upper = np.array([28, 250, 255])

        # Threshold the HSV image to get the colors defined above
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
        kernel = np.ones((4, 4), np.uint8) 
        mask = cv2.inRange(hsv, ball_color_lower, ball_color_upper)
        mask = cv2.dilate(mask, kernel, iterations=4)
        mask = cv2.erode(mask, kernel, iterations=8)
        mask = cv2.dilate(mask, kernel, iterations=5)
        image = cv2.bitwise_not(mask)

        #image = frame
        
        # Set our filtering parameters 
        # Initialize parameter setting using cv2.SimpleBlobDetector 
        params = cv2.SimpleBlobDetector_Params() 
        
        # Set Area filtering parameters 
        params.filterByArea = True
        params.minArea = 500
        params.maxArea = 1000000
        
        # Set Circularity filtering parameters 
        params.filterByCircularity = True 
        params.minCircularity = 0.6
        
        # Set Convexity filtering parameters 
        params.filterByConvexity = True
        params.minConvexity = 0.1
            
        # Set inertia filtering parameters 
        params.filterByInertia = True
        params.minInertiaRatio = 0.01
        
        # Create a detector with the parameters 
        detector = cv2.SimpleBlobDetector_create(params) 
            
        # Detect blobs 
        keypoints = detector.detect(image) 

        # if last_location is None:
        #     continue
        #     #last_location = plate_centre

        if len(keypoints) > 0:
            ball_location = [keypoints[0].pt[0], resolution[1]-keypoints[0].pt[1]]
            last_location = ball_location
            dps+=1
            #print(ball_location)
        else:
            ball_location = last_location
        
        # Draw blobs on our image as red circles 
        blank = np.zeros((1, 1))  
        blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255), 
                                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
        for keypoint in keypoints:
            cv2.circle(blobs, (int(keypoint.pt[0]), int(keypoint.pt[1])), 5, (0, 0, 255), -1)

        number_of_blobs = len(keypoints) 
        text = "Number of Circular Blobs: " + str(len(keypoints)) + "\n Ball Location: " + str(ball_location) 
        cv2.putText(blobs, text, (20, 550), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2) 
        cv2.circle(blobs, plate_centre, 5, (255, 0, 0), -1)
        
        # Show blobs 
        cv2.imshow("Filtering Circular Blobs Only", blobs) 

        # Display the resulting frame
        cv2.imshow('frame', frame)
        #cv2.imshow('mask', mask)


        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv2.destroyAllWindows()

# Stops the thread
def stop():
    global is_running
    is_running = False

# 
