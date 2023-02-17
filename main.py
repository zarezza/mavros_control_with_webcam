import cv2 as cv
import mediapipe as md

#initialize video capture and video size, use default camera
cap = cv.VideoCapture(0) #my default camera resolution = 720x1280

#for hand detection:
hand_detect = md.solutions.hands
#set the parameters of object from mediapipe
hand_param = hand_detect.Hands(max_num_hands = 1, #number of hands that can be detected
                       min_detection_confidence = 0.5) #value for detection confidence, consider an image as succseful or not
draw = md.solutions.drawing_utils #functions to draw landmarks and connections on the video frames

while True:
    _, img = cap.read() #reads the video stream from the camera and return boolean value and the frame
    img = cv.flip(img, 1) #mirror image
    imgrgb = cv.cvtColor(img, cv.COLOR_BGR2RGB) #convert color to RGB, mediapipe library only works with RGB
    res = hand_param.process(imgrgb) #store landmark information

    landmark = [] #for storing landmark coordinates
    tip_id = [4, 8, 12, 16, 20]

    #make list of boudaries for control
    bound_list = ([[0, 0], [1280, 150]],    #top
                  [[0, 570], [1280, 720]],  #bottom
                  [[0, 151], [400, 569]],   #left
                  [[880, 151], [1280, 569]],#right
                  [[400, 151], [880, 569]]) #middle

    overlay = img.copy()
    cv.rectangle(overlay, bound_list[0][0], bound_list[0][1], (0, 0, 255), -1) #top
    cv.rectangle(overlay, bound_list[1][0], bound_list[1][1], (0, 0, 255), -1) #bottom
    cv.rectangle(overlay, bound_list[2][0], bound_list[2][1], (0, 255, 0), -1) #left
    cv.rectangle(overlay, bound_list[3][0], bound_list[3][1], (0, 255, 0), -1) #right
    cv.rectangle(overlay, bound_list[4][0], bound_list[4][1], (255, 0, 0), -1) #middle
    alpha = 0.4
    img = cv.addWeighted(overlay, alpha, img, 1 - alpha, 0)


    if res.multi_hand_landmarks: #true if hand is detected in image
        #iterates over each hand landmarks data and stores the landmarks id and coordinates in the landmark
        for handlm in res.multi_hand_landmarks:
            for id, lm in enumerate(handlm.landmark):
                height, width, _ = img.shape
                cx, cy = int(lm.x * width), int(lm.y * height)
                landmark.append([id, cx, cy]) #store landmark id and coordinates
                if len(landmark) != 0 and len(landmark) == 21: #true if landmark is not empty and 21 landmark detected
                    fingercount = 0

                    #check if its a right hand or left hand
                    if landmark[12][1] > landmark[20][1]: #true if MIDDLE_FINGER_TIP coordinates are greater than PINKY_TIP (left)
                        #print("left hand detected!")
                        if landmark[tip_id[0]][1] > landmark[tip_id[0]-1][1]: #true if THUMB_TIP coordinates are greater than THUMB_IP,
                            fingercount += 1
                    else:
                        #print("right hand detected!")
                        if landmark[tip_id[0]][1] < landmark[tip_id[0]-1][1]:
                            fingercount += 1

                    #counting other finger
                    for id in range (1, 5): #since the THUMB_TIP already found before, skip index 0 and iterate all other finger
                        if landmark[tip_id[id]][2] < landmark[tip_id[id]-2][2]: #true if the coordinates of tip fingers are greater than coordinates of finger pip
                            fingercount += 1

                    print(fingercount)

                #draw points and lines of hand
                draw.draw_landmarks(img,handlm,hand_detect.HAND_CONNECTIONS,draw.DrawingSpec(color=(0,255,0),thickness=2,circle_radius=2),draw.DrawingSpec(color=(0,0,255),thickness=2,circle_radius=3))


    cv.imshow("RESULT",img)

    #press q to quit
    if cv.waitKey(1) == ord('q'):
        break

cv.destroyAllWindows()

