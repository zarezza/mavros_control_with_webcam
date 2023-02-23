import cv2 as cv
import mediapipe as md
import time
import rospy
from geometry_msgs.msg import Twist
from pyzbar.pyzbar import decode

cap = cv.VideoCapture(0) #my default camera resolution = 720x1280
width = int(cap.get(3)) #width
height = int(cap.get(4)) #height

hand_detect = md.solutions.hands
#set the parameters of object from mediapipe
hand_param = hand_detect.Hands(max_num_hands = 1, #number of hands that can be detected
                       min_detection_confidence = 0.5) #value for detection confidence, consider an image as succseful or not
#get functions to draw landmarks and connections on the video frames
draw = md.solutions.drawing_utils

#boundaries for movement control:
top = ([int(width/3), 0], [int(width/3*2), int(height/4)]) #stx, sty, enx, eny, same goes for all
bottom = ([int(width/3), int(height/4*3)], [int(width/3*2), height])
left = ([0, 0], [int(width/3), height])
right = ([int(width/3*2), 0], [width, height])
middle = ([int(width/3)+1, int(height/4)+1], [int(width/3*2)-1, int(height/4*3)-1])

#storing tip id's for later use
tip_id = [4, 8, 12, 16, 20]


def hand_func(handlm, landmark, img):
    for id, lm in enumerate(handlm.landmark):
        height, width, _ = img.shape
        cx, cy = int(lm.x * width), int(lm.y * height)
        landmark.append([id, cx, cy])
        if len(landmark) != 0 and len(landmark) == 21: #true if landmark is not empty and 21 landmark detected
            finger_count = 0

            #check if its a right hand or left hand
            if landmark[12][1] > landmark[20][1]: #true if MIDDLE_FINGER_TIP coordinates are greater than PINKY_TIP (left)
                #print("left hand detected!")
                if landmark[tip_id[0]][1] > landmark[tip_id[0]-1][1]: #true if THUMB_TIP coordinates are greater than THUMB_IP,
                    finger_count += 1
            else:
                #print("right hand detected!")
                if landmark[tip_id[0]][1] < landmark[tip_id[0]-1][1]:
                    finger_count += 1

            #count other finger
            for id in range (1, 5): #since the THUMB_TIP already found before, skip index 0 and iterate all other finger
                if landmark[tip_id[id]][2] < landmark[tip_id[id]-2][2]: #true if the coordinates of tip fingers are greater than coordinates of finger pip
                    finger_count += 1

            draw.draw_landmarks(img,handlm,hand_detect.HAND_CONNECTIONS,draw.DrawingSpec(color=(0,255,0),thickness=2,circle_radius=2),draw.DrawingSpec(color=(0,0,255),thickness=2,circle_radius=3))
            return landmark, finger_count

yaw_chace = 0
yaw_rate = 0.2
speed_rate = 3.0
def movement(x, y, z, ang):
    global yaw_chace
    temp_msg = Twist()
    
    temp_msg.linear.x = x
    temp_msg.linear.y = y
    temp_msg.linear.z = z
    temp_msg.angular.z = yaw_chace 
    yaw_chace = yaw_chace + ang
    
    if yaw_chace > 8.25:
        yaw_chace = -8.25
    elif yaw_chace < -8.25:
        yaw_chace = 8.25

    return temp_msg
    
def control_xz(img, landmark, lable_chace):

    res = Twist()
    cx = landmark[9][1]
    cy = landmark[9][2]

    img = cv.circle(img, (cx, cy), 20, (0, 255, 255), -1)
    overlay = img.copy()
    alpha = 0.2

    cv.rectangle(overlay, top[0], top[1], (0, 0, 100), -1)
    cv.rectangle(overlay, bottom[0], bottom[1], (0, 0, 100), -1)
    cv.rectangle(overlay, left[0], left[1], (0, 100, 0), -1)
    cv.rectangle(overlay, right[0], right[1], (0, 100, 0), -1)
    cv.rectangle(overlay, middle[0], middle[1], (100, 0, 0), -1)
    img = cv.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    if cx > top[0][0] and cx < top[1][0] and cy > top[0][1] and cy < top[1][1]: 
        cv.rectangle(overlay, top[0], top[1], (0, 0, 255), -1)
        res = movement(speed_rate, 0.0, 0.0, 0.0) 
        lable = "forward"
    elif cx > bottom[0][0] and cx < bottom[1][0] and cy > bottom[0][1] and cy < bottom[1][1]:
        cv.rectangle(overlay, bottom[0], bottom[1], (0, 0, 255), -1)
        res = movement(-speed_rate, 0.0, 0.0, 0.0) 
        lable = "backward"
    elif cx > left[0][0] and cx < left[1][0] and cy > left[0][1] and cy < left[1][1]:
        cv.rectangle(overlay, left[0], left[1], (0, 255, 0), -1)
        res = movement(0.0, 0.0, 0.0, yaw_rate) 
        lable = "yaw_left"
    elif cx > right[0][0] and cx < right[1][0] and cy > right[0][1] and cy < right[1][1]:
        cv.rectangle(overlay, right[0], right[1], (0, 255, 0), -1)
        res = movement(0.0, 0.0, 0.0, -yaw_rate) 
        lable = "yaw_right"
    elif cx > middle[0][0] and cx < middle[1][0] and cy > middle[0][1] and cy < middle[1][1]:
        cv.rectangle(overlay, middle[0], middle[1], (255, 0, 0), -1)
        res = movement(0.0, 0.0, 0.0, 0.0)
        lable = "idle"
    else: 
        res = movement(0.0, 0.0, 0.0, 0.0)
        lable = "idle"

    img = cv.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    if lable != lable_chace:
        print(rospy.get_caller_id() + ": " + lable)

    return img, res, lable

def control_yz(img, landmark, lable_chace):
    
    res = Twist()
    cx = landmark[9][1]
    cy = landmark[9][2]

    img = cv.circle(img, (cx, cy), 20, (0, 255, 255), -1)
    overlay = img.copy()
    alpha = 0.2

    cv.rectangle(overlay, top[0], top[1], (0, 100, 0), -1)
    cv.rectangle(overlay, bottom[0], bottom[1], (0, 100, 0), -1)
    cv.rectangle(overlay, left[0], left[1], (0, 0, 100), -1)
    cv.rectangle(overlay, right[0], right[1], (0, 0, 100), -1)
    cv.rectangle(overlay, middle[0], middle[1], (100, 0, 0), -1)
    img = cv.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    if cx > top[0][0] and cx < top[1][0] and cy > top[0][1] and cy < top[1][1]: 
        cv.rectangle(overlay, top[0], top[1], (0, 255, 0), -1)
        res = movement(0.0, 0.0, speed_rate, 0.0) 
        lable = "upward"
    elif cx > bottom[0][0] and cx < bottom[1][0] and cy > bottom[0][1] and cy < bottom[1][1]:
        cv.rectangle(overlay, bottom[0], bottom[1], (0, 255, 0), -1)
        res = movement(0.0, 0.0, -speed_rate, 0.0) 
        lable = "downward"
    elif cx > left[0][0] and cx < left[1][0] and cy > left[0][1] and cy < left[1][1]:
        cv.rectangle(overlay, left[0], left[1], (0, 0, 255), -1)
        res = movement(0.0, speed_rate, 0.0, 0.0) 
        lable = "left"
    elif cx > right[0][0] and cx < right[1][0] and cy > right[0][1] and cy < right[1][1]:
        cv.rectangle(overlay, right[0], right[1], (0, 0, 255), -1)
        res = movement(0.0, -speed_rate, 0.0, 0.0) 
        lable = "right"
    elif cx > middle[0][0] and cx < middle[1][0] and cy > middle[0][1] and cy < middle[1][1]:
        cv.rectangle(overlay, middle[0], middle[1], (255, 0, 0), -1)
        res = movement(0.0, 0.0, 0.0, 0.0)
        lable = "idle"
    else: 
        res = movement(0.0, 0.0, 0.0, 0.0)
        lable = "idle"

    img = cv.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    if lable != lable_chace:
        print(rospy.get_caller_id() + ": " + lable)

    return img, res, lable

loiter_chace = 0
def loiter_square(lable_chace):
    res = Twist()
    global loiter_chace
    lable = "loiter_square"
    if loiter_chace < 100:
        res = movement(speed_rate, 0.0, 0.0, 0.0)
    elif loiter_chace >= 100 and loiter_chace < 200:
        res = movement(0.0, speed_rate, 0.0, 0.0)
    elif loiter_chace >= 200 and loiter_chace < 300:
        res = movement(-speed_rate, 0.0, 0.0, 0.0)
    elif loiter_chace <= 300 and loiter_chace > 400:
        res = movement(0.0, -speed_rate, 0.0, 0.0)
    
    loiter_chace = loiter_chace + 1
    
    if loiter_chace >= 400:
        loiter_chace = 0
    print("loiter: " + str(loiter_chace))
    if lable != lable_chace:
        print(rospy.get_caller_id() + ": " + lable)
    
    return res, lable



if __name__ == '__main__':
    rospy.init_node('hand_cam',)
    vel_pub = rospy.Publisher('velocity_commands', Twist, queue_size=10)
    vel_msg = Twist()


    rate = rospy.Rate(30)
    

    lable = "" #used for chace 
    finger_count = -1
    time_init = 0
    while not rospy.is_shutdown():
        _, img = cap.read() #reads the video stream from the camera and return boolean value and the frame
        img = cv.flip(img, 1) #mirror image
        imgrgb = cv.cvtColor(img, cv.COLOR_BGR2RGB) #convert color to RGB, mediapipe library only works with RGB
        res = hand_param.process(imgrgb) #store landmark information

        landmark = []

        if res.multi_hand_landmarks: #true if hand is detected in image
            #iterates over each hand landmarks data and stores the landmarks id and coordinates in the landmark
            for handlm in res.multi_hand_landmarks:
                landmark, finger_chace = hand_func(handlm, landmark, img)

            if finger_count != finger_chace: #to keep the function not changed imidietly
                if time.time() - time_init > 0.7: #hold 0.7 sec if finger count changed value 
                    finger_count = finger_chace
            else:
                time_init = time.time() #restart the timer


            
            if finger_count == 0:
                img, vel_msg, lable = control_xz(img, landmark, lable)
            elif finger_count == 1:
                img, vel_msg, lable = control_yz(img, landmark, lable)
            elif finger_count == 2:
                movement(0.0, 0.0, 0.0, 0.0)
            elif finger_count == 3:
                movement(0.0, 0.0, 0.0, 0.0)
            elif finger_count == 4:
                vel_msg, lable = loiter_square(lable)
            elif finger_count == 5:
                movement(0.0, 0.0, 0.0, 0.0)

            # rospy.loginfo(str_pub)
            vel_pub.publish(vel_msg)

        cv.imshow("RESULT",img)

        #press q to quit
        if cv.waitKey(1) == ord('q'):
            break

        rate.sleep()
    cv.destroyAllWindows()
