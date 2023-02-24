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
hand_param = hand_detect.Hands(max_num_hands = 2, #number of hands that can be detected
                       min_detection_confidence = 0.5) #value for detection confidence, consider an image as succseful or not
#get functions to draw landmarks and connections on the video frames

draw = md.solutions.drawing_utils
#storing tip id's for later use
tip_id = [4, 8, 12, 16, 20]

middle_right = [int(width/4*3), int(height/2)]
middle_left = [int(width/4), int(height/2)]

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

def process_landmark(handlm, landmark):
    for id, lm in enumerate(handlm.landmark):
        cx, cy = int(lm.x * width), int(lm.y * height)
        landmark.append([id, cx, cy])
    return landmark

def process_finger(landmark):
    finger_count = 0
    for id in range(1, 5):
        # print(landmark[tip_id[id]][0]) 
        if landmark[tip_id[id]][2] < landmark[tip_id[id]-2][2]:
            finger_count += 1
    return finger_count
    
def midpoint(ax, ay, bx, by):
    res = [0, 0]
    res[0] = int((ax + bx)/2)
    res[1] = int((ay + by)/2)

    return res

if __name__ == '__main__':
    rospy.init_node('hand_cam',)
    vel_pub = rospy.Publisher('velocity_commands', Twist, queue_size=10)
    vel_msg = Twist()

    ros_rate = rospy.Rate(30)

    finger_count = -1
    time_init = time.time()
    while not rospy.is_shutdown():
        _, img = cap.read() #reads the video stream from the camera and return boolean value and the frame
        img = cv.flip(img, 1) #mirror image
        imgrgb = cv.cvtColor(img, cv.COLOR_BGR2RGB) #convert color to RGB, mediapipe library only works with RGB
        res = hand_param.process(imgrgb) #store landmark information
        
        landmark = []
        ref_right = [0, 0, 0]
        ref_left = [0, 0, 0]
        x, y, z, ang_z = 0, 0, 0, 0
        cv.circle(img, middle_left, 20, (0, 255, 255), -1)
        cv.circle(img, middle_right, 20, (0, 255, 0), -1)
        if res.multi_hand_landmarks: #true if hand is detected in image
            #iterates over each hand landmarks data and stores the landmarks id and coordinates in the landmark
            # print(res.multi_hand_landmarks) #prints all landmark without len
            hand_count = 0
            for handlm in res.multi_hand_landmarks:
                draw.draw_landmarks(img,handlm,hand_detect.HAND_CONNECTIONS,draw.DrawingSpec(color=(0,255,0),thickness=2,circle_radius=1),draw.DrawingSpec(color=(0,0,255),thickness=2,circle_radius=1))
                landmark = process_landmark(handlm, landmark)
            if len(landmark) == 42:
                if landmark[20][1] > width/2 and landmark[41][1] < width/2:
                    ref_right = landmark[:len(landmark)//2]
                    ref_left = landmark[len(landmark)//2:]                  
                else:
                    ref_right = landmark[len(landmark)//2:]
                    ref_left = landmark[:len(landmark)//2]


                if process_finger(ref_right) == 0:
                    mid_palm_right = midpoint(ref_right[1][1], ref_right[1][2], ref_right[13][1], ref_right[13][2])
                    cv.circle(img, mid_palm_right, 20, (0, 0, 0), -1)

                    x = (middle_right[1] - mid_palm_right[1]) * 0.05
                    ang_z = (middle_right[0] - mid_palm_right[0]) * 0.003
                    
                if process_finger(ref_left) == 0:
                    mid_palm_left = midpoint(ref_left[1][1], ref_left[1][2], ref_left[13][1], ref_left[13][2])
                    cv.circle(img, mid_palm_left, 20, (0, 0, 0), -1)

                    y = (middle_left[0] - mid_palm_left[0]) * 0.05
                    z = (middle_left[1] - mid_palm_left[1]) * 0.05
                    
                print("x:\t%.2f" % x)
                print("y:\t%.2f" % y)
                print("z:\t%.2f" % z)
                print("ang_z:\t%.2f" % ang_z + "\n") 
                vel_msg = movement(x, y, z, ang_z)
            else: vel_msg = movement(x, y, z, ang_z)
            vel_pub.publish(vel_msg)
        cv.imshow("RESULT",img)

        #press q to quit
        if cv.waitKey(1) == ord('q'):
            break

        ros_rate.sleep()
    cv.destroyAllWindows()