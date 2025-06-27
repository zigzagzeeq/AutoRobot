import tensorflow as tf
import numpy as np
import os
import rospy

path = os.path.dirname(os.path.abspath(__file__))
# Note: Suggesting to have 5-10 seconds delay before the first prediction to allow the model to warm up
class FacialEmotionDetector:
    
    model_path = os.path.join(path, "test_auc_0.89.keras")  # Update this path to your model file

    def __init__(self):
        rospy.init_node('robot_faceEmotion_detector', anonymous=False)
        self.model = tf.keras.models.load_model(self.model_path)
        self.emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']
        self.emotion_freq = {label: 0 for label in self.emotion_labels}


    def preprocess_face(self, face_image):
        # Resize the face image to the input size expected by the model
        face_image = tf.image.resize(face_image, (48, 48))
        # Normalize the pixel values to [0, 1]
        face_image = face_image / 255.0
        face_image = face_image.reshape(1, 48, 48, 1)
        # Expand dimensions to match model input shape (batch_size, height, width, channels)
        face_image = np.expand_dims(face_image, axis=0)
        return face_image

    def update_emotion_frequency(self, emotion):
        if emotion in self.emotion_freq:
            self.emotion_freq[emotion] += 1
        else:
            self.emotion_freq[emotion] = 1
    
    def predict_emotion(self, face_image):
        # Preprocess the image for the model
        face_image = self.preprocess_face(face_image)
        predictions = self.model.predict(face_image)
        predicted_index = np.argmax(predictions[0])
        predicted_emotion = self.emotion_labels[predicted_index]
        self.update_emotion_frequency(predicted_emotion)
        return self.emotion_labels[predicted_index]
    
    def get_emotion_frequency(self):
        return self.emotion_freq