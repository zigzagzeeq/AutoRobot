import rospy
from std_msgs.msg import String
from SpeakListen import SpeakListen
from ActivityDemo import ActivityDemo
from RobotResponse import RobotResponse

class SpeechRobot:
    """Class to handle speech functionalities of the robot"""

    def __init__(self, publisherNodeName, subcriberNodeName, topicName):
        self.speak_listen = SpeakListen(publisherNodeName, subcriberNodeName, topicName)
        self.activity_demo = ActivityDemo(publisherNodeName, subcriberNodeName, topicName)
        self.robot_response = RobotResponse(publisherNodeName, subcriberNodeName, topicName)

    # TODO: Implement the main logic for the speech functionalities of the robot.
    # 1. The robot receives a message containing the list of emotions detected by the vision system along with the frequency.
    # 2. The robot generates a response based on the detected emotions from RobotResponse.
    # 3. The robot speaks the response to the user from SpeakListen.
    # 4. The robot listens for the user's response from SpeakListen.
    # 5. The robot processes the user's response on LLM and suggest 1 activity of the related emotion.
    # 6. The robot speaks the suggested activity to the user from SpeakListen.
    # 7. The robot listens for the user's decision on whether to proceed with the suggested activity from SpeakListen.
        # If the user agrees, the robot initiates the activity demonstration from ActivityDemo.
        # If the user disagrees, the robot acknowledges the decision and offers to assist with anything else.
    # 8. The robot continues to run and listen for further interactions. (KIV - can change if not achievable)

    def run(self):
        """Run the speech functionalities of the robot"""
        rospy.loginfo("SpeechRobot is running...")

        rospy.spin()  # Keep the node running

