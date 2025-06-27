import rospy

# Intialize the ROS node
rospy.init_node('robot_main_function', anonymous=False)

# Import the necessary modules
from robot.FacialDetector import FacialDetector
from robot.EmotionDetector import FacialEmotionDetector

# Initialize the facial detector and emotion detector
facial_detector = FacialDetector()
emotion_detector = FacialEmotionDetector()

def main():
    rospy.loginfo("Robot main function started.")
    
    # Start the facial detection and emotion detection processes
    facial_detector.start_detection()
    emotion_detector.start_detection()

    rospy.loginfo("Robot is now detecting faces and emotions.")

    # Keep the node running until it is shut down
    rospy.spin()