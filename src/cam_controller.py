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

#storing tip id's for later use
tip_id = [4, 8, 12, 16, 20]


def hand_func(handlm, landmark, img):
    for id, lm in enumerate(handlm.landmark):
        height, width, _ = img.shape
        cx, cy = int(lm.x * width), int(lm.y * height)
        landmark.append([id, cx, cy])
        if len(landmark) != 0 and len(landmark) == 21: #true if landmark is not empty and 21 landmark detected
            finger_count = 0

            #check if its a right hand or left hand (not quite right)
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
yaw_rate = 0.04
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

def control(img, landmark, capt, mode):
    ref_c = (landmark[9][1], landmark[9][2])

    img = cv.line(img, ref_c, capt, (0, 0, 0), 20)
    img = cv.circle(img, capt, 20, (0, 255, 255), -1)
    img = cv.circle(img, ref_c, 20, (0, 255, 255), -1)

    speed = [0.0, 0.0, 0.0, 0.0]

    if mode == 0:
        speed[0] = (capt[1] - ref_c[1]) * 0.01
        speed[1] = 0
        speed[2] = 0
        speed[3] = (capt[0] - ref_c[0]) * 0.01
        res = movement(speed)
    elif mode == 1:
        speed[0] = 0
        speed[1] = (capt[0] - ref_c[0]) * 0.01
        speed[2] = (capt[1] - ref_c[1]) * 0.01
        speed[3] = 0
        res = movement(speed)
    return img, res

if __name__ == '__main__':
    rospy.init_node('hand_cam',)
    vel_pub = rospy.Publisher('velocity_commands', Twist, queue_size=10)
    vel_msg = Twist()

    ros_rate = rospy.Rate(30)

    finger_count = -1
    time_init = time.time()
    capt = []
    capt_chace = []
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
                    if finger_count == 0:
                        capt = (landmark[9][1], landmark[9][2])
                        capt_chace = capt
            else:
                time_init = time.time() #restart the timer


            if finger_count == 0:
                img, vel_msg, speed = control(img, landmark, speed, capt, finger_count)
            elif finger_count == 1:
                img, vel_msg, speed = control(img, landmark, speed, capt, finger_count)
            else: vel_msg = movement(0.0, 0.0, 0.0, 0.0)

            # rospy.loginfo(str_pub)
            vel_pub.publish(vel_msg)

        cv.imshow("RESULT",img)

        #press q to quit
        if cv.waitKey(1) == ord('q'):
            break

        ros_rate.sleep()
    cv.destroyAllWindows()
