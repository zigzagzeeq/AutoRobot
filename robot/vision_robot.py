import cv2
import numpy as np
import tensorflow as tf

# Load your trained Keras model
# If need to change different model file, perform the changes in the following Kaggle link:
# https://colab.research.google.com/#fileId=https%3A//storage.googleapis.com/kaggle-colab-exported-notebooks/warikanori/fer13-cnn-custom-model-using-keras.f76fcac8-e13d-4390-85f6-7498c3eb69ac.ipynb%3FX-Goog-Algorithm%3DGOOG4-RSA-SHA256%26X-Goog-Credential%3Dgcp-kaggle-com%2540kaggle-161607.iam.gserviceaccount.com/20250626/auto/storage/goog4_request%26X-Goog-Date%3D20250626T173857Z%26X-Goog-Expires%3D259200%26X-Goog-SignedHeaders%3Dhost%26X-Goog-Signature%3D2a26d858166221ad97f0a9fcddc5f6cc32eeaf50687c82689cd6d32bed9c87f9bb2a6748d33482e1a80cd5f96f4c61ae686039226d07f0dee48af29d849ebb94426a53dd44e58bd75191de186acf77d6b1fe3ee4374c1d61ac5535b24d55c7f5466db28a084bb2ce294d413c0dfa7e06d6eff08b7786d2b76808626ceb1dd98aa63ec485bfc11a784b79895e0bc88942e19e392aa165eb174bcff004383a8d1a8dc150ab31e712d179c71b0b7bd5411695b455de6e9cdc0a3a8b4a7cdf4904c2d6f672941b9a80e5117f696717fb206668b8ddf5c79893dbe410138f533abfc1122fdebebd7f5fd03fb6f6aec7b56d43973a255fc5e88d2174f3b93b51be732f
# Please request Ikram if need to change the model file
model = tf.keras.models.load_model("robot\\test_auc_0.89.keras")

# Emotion labels (adjust to match your model's output order)
emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']

# Load face detector (you can also use MTCNN if you prefer)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Start webcam
cap = cv2.VideoCapture(0) # Change to 1 if want to use external webcam, 0 for laptop webcam

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