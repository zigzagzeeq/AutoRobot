import rospy

# Intialize the ROS node
rospy.init_node('main_function', anonymous=False)

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

# 1. Activate the robot
# 2. Start the main function
# 3. Detect any faces in the camera feed
# 4. For 5-10 seconds, Detect the emotion of the detected faces and get all the emotions frequency detected
# 5. Generate response based on the detected emotions which one is the highest freq,
#    if there is a tie, robot respond with mixed emotions present in the camera and pick randomly among the tied emotions
#    if there is a tie and in the tie has neutral emotions, the neutral emotion is ignored and only focus other tied emotions
#    if no emotion detected or (neutral emotion is the highest and no tie), go back to step 3
# 6. Robot ask the user what made him/her feel that way
# 7. The user responds with voice (Speech Recognition)
# 8. Robot responds with a text response based on the user's input (LLM small response and suggestion)
# 9. Robot ask the user if he/she wants to continue the conversation or assistance (ie like breathing exercise, meditation, etc.)
#    if the user says yes, the robot will demostrate the suggested activity
#    else end the program / go back to step 3

