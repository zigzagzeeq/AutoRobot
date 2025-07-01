import cv2

class FacialDetector:
    def __init__(self, publisherNodeName, subcriberNodeName, topicName) -> None:
        rospy.init_node(publisherNodeName)
        rospy.Subscriber(subcriberNodeName, Image, image_callback)
        self.pub = rospy.Publisher(topicName, String, queue_size=60)
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    def detect_faces(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
        return faces
    
    def draw_faces(self, frame, faces):
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return frame
    
    def process_frame(self, frame):
        faces = self.detect_faces(frame)
        frame_with_faces = self.draw_faces(frame, faces)
        return frame_with_faces, faces

