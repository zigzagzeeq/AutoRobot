import rospy
import time
import cv2

# Intialize the ROS node
rospy.init_node('main_function', anonymous=False)

# Import the necessary modules
from robot.FacialDetector import FacialDetector
from robot.EmotionDetector import FacialEmotionDetector

# Initialize the facial detector and emotion detector
facial_detector = FacialDetector()
emotion_detector = FacialEmotionDetector()


time.sleep(5)


def main():
    cap = cv2.VideoCapture(0) # Change to 1 if want to use external webcam, 0 for laptop webcam

    if not cap.isOpened():
        rospy.logerr("Could not open video device")
        return
    rospy.loginfo("Video device opened successfully.")

    # Main loop for detecting faces and emotions
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to read frame from video device")
            break
        
        # Detect faces in the frame
        faces = facial_detector.detect_faces(frame)
        emotion_freq = dict()

        if len(faces) > 0:
            rospy.loginfo(f"Detected {len(faces)} face(s).")
            for (x, y, w, h) in faces:
                face = frame[y:y + h, x:x + w]
                emotion = emotion_detector.predict_emotion(face)
                if emotion not in emotion_freq:
                    emotion_freq[emotion] = 0
                emotion_freq[emotion] += 1
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        else:
            rospy.loginfo("No faces detected.")

        # Display the frame with detected faces and emotions
        cv2.imshow('Emotion Detection - Press Q to Quit', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    rospy.loginfo("Robot main function started.")

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

