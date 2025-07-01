import cv2
import numpy as np
import tensorflow as tf
from EmotionDetector import FacialEmotionClassifier
from FacialDetector import FacialDetector
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy

def image_callback(data):
    """
    Callback function to process incoming ROS Image messages.
    Converts the ROS Image to OpenCV format and processes it.
    """
    try:
        # Convert ROS Image to OpenCV format
        cv_image = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)
        
        # Process the image (e.g., detect faces, emotions)
        # This part can be customized based on your requirements
        rospy.loginfo("Received an image for processing.")
        
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

# TODO: Make connection between the camera-robot to VisionRobot, and VisionRobot to SpeechRobot
# 1. VisionRobot captures images from the robot camera
# 2. The frames received are processesed.
# 3. The processed frames detect any faces in the camera.
# 4. The faces put into a emotion classifier model.
# 5. The model predicts the emotion of the faces.
# 6. All the predicted emotions are stored in a dictionary with the frequency of the emotion.
# 7. The dictionary is published to the SpeechRobot.

class VisionRobot:
    def __init__(self, publisherNodeName, subcriberNodeName, topicName) -> None:
        """
            def main():
                # Declare a global publisher variable
                global pub
                # Create a node with the name 'face_recognition_node'
                rospy.init_node('face_recognition_node', anonymous=True)
                # Subscribe to the '/usb_cam/image_raw' topic which receives raw images from the USB camera
                rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
                # Create a publisher that will publish messages to the '/recognized_face' topic
                pub = rospy.Publisher('/recognized_face', String, queue_size=10)
                # Keep the node running and processing callbacks
                rospy.spin()
        """
        self.emotion_dict = dict()
        self.emotion_classifier = FacialEmotionClassifier()
        self.facial_detector = FacialDetector()
        rospy.init_node(publisherNodeName)
        rospy.Subscriber(subcriberNodeName, Image, image_callback)
        self.pub = rospy.Publisher(topicName, String, queue_size=60)
    
    def run(self):
        rospy.loginfo("VisionRobot is running...")
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            rospy.logwarn("Warn: Could not open webcam.")
        
        while True:
            self.emotion_dict = dict()
            _, frame = cap.read()
            frame_with_faces, faces = self.facial_detector.process_frame(frame)

            for (x, y, w, h) in faces:
                face_image = frame[y:y + h, x:x + w]
                emotion, confidence = self.emotion_classifier.predict_emotion(face_image)
                if emotion not in self.emotion_dict:
                    self.emotion_dict[emotion] = 1
                else:
                    self.emotion_dict[emotion] += 1
                
                # Draw box and label
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = f"{emotion} ({confidence*100:.1f}%)"
                cv2.putText(frame, text, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow('Facial Emotion Detection - Press Q to Quit', frame_with_faces)
            print("Emotion Dictionary:", self.get_emotion_dict())
            rospy.loginfo(f"str({self.get_emotion_dict()})")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    
    def get_emotion_dict(self):
        return self.emotion_dict





