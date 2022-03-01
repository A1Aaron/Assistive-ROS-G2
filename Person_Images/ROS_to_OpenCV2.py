# Required python packages and ROS messages
from json import detect_encoding
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sys
import rospy
import cv2 
from std_msgs.msg import String
from sensor_msgs.msg import Image




class image_converter:
    
     
    
    
    
     def __init__(self):
         
        
    
        # Creates a Node called image_converter
        rospy.init_node('image_converter', anonymous=True)
        
        # Creates a new publisheer object on the 'image_topic_2 topic' 
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        
        # Cv_Bridge initalised
        self.bridge = CvBridge()
        
        
        # Subcribes to the current image topic in gazeboo for the turtlebot and stores the daata in callback
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        
        
     
     
        
        
     def callback(self,data):
        try:
            # Gets the image from the subscriber and converts to OpenCv readable image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
         
        def empty(a):
            pass 
    
        # Create a Trackbar
        cv2.namedWindow("Result")
        cv2.resizeWindow("Result",640,480)
        cv2.createTrackbar("Scale","Result",50,1000,empty)
        cv2.createTrackbar("Neig","Result",8,20,empty)
        cv2.createTrackbar("Min Area","Result",0,100000,empty) 
        
    
        
        # Setting up additional paramentes to parse from Trackbar
        neig = cv2.getTrackbarPos("Neig","Result")
        scaleVal = 1 + (cv2.getTrackbarPos("Scale","Result")/1000)
         
        # Trained model object    
        cascade_person = cv2.CascadeClassifier('/home/aaron/catkin_ws/src/opencv_interface/src/Person_Images/cascade/cascade.xml')
        #cascade_person = cv2.CascadeClassifier('/home/aaron/opencv-3.4.16/data/haarcascades/haarcascade_lowerbody.xml')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Do object detection and return image rectangles
        objects = cascade_person.detectMultiScale(cv_image,scaleVal,neig)
        #objects = cascade_person.detectMultiScale(cv_image)
    
        # If an object is detected draw a box with classId and boundingbox
        for (x,y,w,h) in objects:
            area = w*h
            minArea = cv2.getTrackbarPos("Min Area","Result")
            if area > minArea:
                detect =+ 1
                cv2.rectangle  (cv_image,(x,y),(x+w,y+h),color=(0,255,0),thickness=1 )
                cv2.putText(cv_image,'Ron',(x,y),cv2.FONT_HERSHEY_COMPLEX,2,(0,255,0),1)
                rospy.loginfo("Detected Ron")
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        
        # # Creates an empty array of Class Names
        # classNames = []
        # # Sources the class name file obtained from OpenCv and inputs it into the empty array above
        # classFile = '/home/aaron/catkin_ws/src/opencv_interface/scripts/coco.names'
        # with open(classFile, 'rt') as f:
        #     classNames = f.read().rstrip('\n').split('\n')
        
        # # Sources the wieghts and config file of the coco names dataset
        # configPath = '/home/aaron/catkin_ws/src/opencv_interface/scripts/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        # weightsPath = '/home/aaron/catkin_ws/src/opencv_interface/scripts/frozen_inference_graph.pb'
        
        # # Creates the detection model default values used        
        # net = cv2.dnn_DetectionModel(weightsPath,configPath)
        # net.setInputSize(320,320)
        # net.setInputScale(1.0/ 127.5)
        # net.setInputMean((127.5, 127.5, 127.5))
        # net.setInputSwapRB(True)
        
        # # Variables to store the ClassID Confidence and bounding box with a 50% detectionthreshold
        # classIds, confs, bbox = net.detect(cv_image,confThreshold=0.5)
        
        # # If an object is detected draw a box with classId and boundingbox
        # if len(classIds) !=0:
        #     for classId, confidence, box in zip(classIds.flatten(),confs.flatten(),bbox):
        #         cv2.rectangle(cv_image,box,color=(0,255,0), thickness =2)
        #         cv2.putText(cv_image,classNames[classId-1],(box[0]+10,box[1]+30),cv2.FONT_HERSHEY_COMPLEX,2,(0,255,0),1)
            
        # # Open a new window and display image
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(1)
        
        # Publish the image data onto the ROS "image_topic_2"
        try:
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
           print(e)
 
  
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
    main(sys.argv)
            
            