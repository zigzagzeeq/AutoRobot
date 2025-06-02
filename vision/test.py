import cv2
import numpy as np
import tensorflow as tf

# Load your trained Keras model
# If need to change different model file, perform the changes in the following Kaggle link:
# https://www.kaggle.com/code/warikanori/fer13-cnn-custom-model-using-keras/edit
# Please request Ikram if need to change the model file
model = tf.keras.models.load_model("test_acc_0.89.keras")

# Emotion labels (adjust to match your model's output order)
emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']

# Load face detector (you can also use MTCNN if you prefer)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Start webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale for face detection and model input
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

    for (x, y, w, h) in faces:
        face = gray[y:y + h, x:x + w]
        face_resized = cv2.resize(face, (48, 48))
        face_normalized = face_resized / 255.0
        face_reshaped = face_normalized.reshape(1, 48, 48, 1)

        # Predict emotion
        preds = model.predict(face_reshaped, verbose=0)
        emotion = emotion_labels[np.argmax(preds)]
        confidence = np.max(preds)

        # Draw box and label
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        text = f"{emotion} ({confidence*100:.1f}%)"
        cv2.putText(frame, text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # Show the result
    cv2.imshow('Emotion Detection - Press Q to Quit', frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
