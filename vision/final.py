import cv2
from fer import FER

# Initialize the emotion detector with MTCNN (good for multiple faces)
emotion_detector = FER(mtcnn=True)

# Start webcam capture
cap = cv2.VideoCapture(0)  # Use 0 for default webcam

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect emotions for all faces in the frame
    results = emotion_detector.detect_emotions(frame)

    for face in results:
        (x, y, w, h) = face["box"]
        emotions = face["emotions"]
        top_emotion = max(emotions, key=emotions.get)

        # Draw bounding box around face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Put the emotion label on the image
        text = f"{top_emotion} ({emotions[top_emotion]*100:.1f}%)"
        cv2.putText(frame, text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Display the result frame
    cv2.imshow('Emotion Recognition - Press Q to Quit', frame)

    # Break on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()