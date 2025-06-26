import tensorflow as tf
import numpy as np


class FaceEmotionDetector:
    def __init__(self, model_path):
        self.model = tf.keras.models.load_model(model_path)
        self.emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']
        self.face_image_list = []
    
    def set_camera(self, camera):
        self.camera = camera
    
    def add_face_image(self, face_image):
        

    def preprocess_face(self, face_image):
        # Resize the face image to the input size expected by the model
        face_image = tf.image.resize(face_image, (48, 48))
        # Normalize the pixel values to [0, 1]
        face_image = face_image / 255.0
        # Expand dimensions to match model input shape (batch_size, height, width, channels)
        face_image = np.expand_dims(face_image, axis=0)
        return face_image

    def predict_emotion(self, face_image):
        # Preprocess the image for the model
        face_image = np.expand_dims(face_image, axis=0)  # Add batch dimension
        predictions = self.model.predict(face_image)
        predicted_index = np.argmax(predictions[0])
        return self.emotion_labels[predicted_index]
    


# Will not be included in the final code, just for testing purposes
def testing():
    import cv2
    detector = FaceEmotionDetector("test_auc_0.89.keras")
    # Assuming you have a face image loaded as a numpy array
    # face_image = ... (load your face image here)
    # emotion = detector.predict_emotion(face_image)
    # print(f"Predicted emotion: {emotion}")