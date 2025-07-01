import tensorflow as tf
import numpy as np
import os
import cv2
import rospy

path = os.path.dirname(os.path.abspath(__file__))
# Note: Suggesting to have 5-10 seconds delay before the first prediction to allow the model to warm up
class FacialEmotionClassifier:
    # Load your trained Keras model
    # If need to change different model file, perform the changes in the following Kaggle link:
    # https://colab.research.google.com/#fileId=https%3A//storage.googleapis.com/kaggle-colab-exported-notebooks/warikanori/fer13-cnn-custom-model-using-keras.f76fcac8-e13d-4390-85f6-7498c3eb69ac.ipynb%3FX-Goog-Algorithm%3DGOOG4-RSA-SHA256%26X-Goog-Credential%3Dgcp-kaggle-com%2540kaggle-161607.iam.gserviceaccount.com/20250626/auto/storage/goog4_request%26X-Goog-Date%3D20250626T173857Z%26X-Goog-Expires%3D259200%26X-Goog-SignedHeaders%3Dhost%26X-Goog-Signature%3D2a26d858166221ad97f0a9fcddc5f6cc32eeaf50687c82689cd6d32bed9c87f9bb2a6748d33482e1a80cd5f96f4c61ae686039226d07f0dee48af29d849ebb94426a53dd44e58bd75191de186acf77d6b1fe3ee4374c1d61ac5535b24d55c7f5466db28a084bb2ce294d413c0dfa7e06d6eff08b7786d2b76808626ceb1dd98aa63ec485bfc11a784b79895e0bc88942e19e392aa165eb174bcff004383a8d1a8dc150ab31e712d179c71b0b7bd5411695b455de6e9cdc0a3a8b4a7cdf4904c2d6f672941b9a80e5117f696717fb206668b8ddf5c79893dbe410138f533abfc1122fdebebd7f5fd03fb6f6aec7b56d43973a255fc5e88d2174f3b93b51be732f
    # Please request Ikram if need to change the model file
    model_path = os.path.join(path, "test_auc_0.89.keras")  # Update this path to your model file

    def __init__(self, publisherNodeName, subcriberNodeName, topicName) -> None:
        # rospy.init_node('robot_faceEmotion_detector', anonymous=False)
        rospy.init_node(publisherNodeName)
        rospy.Subscriber(subcriberNodeName, Image, image_callback)
        self.pub = rospy.Publisher(topicName, String, queue_size=60)
        self.model = tf.keras.models.load_model(self.model_path)
        self.emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']

    def preprocess_face(self, face_image):
        face_resized = cv2.resize(face_image, (48, 48))
        face_normalized = face_resized / 255.0
        face_reshaped = face_normalized.reshape(1, 48, 48, 1)

        return face_reshaped
    
    def predict_emotion(self, face_image):
        # Preprocess the image for the model
        face_image = self.preprocess_face(face_image)
        predictions = self.model.predict(face_image, verbose = 0)
        predicted_index = np.argmax(predictions[0])
        return self.emotion_labels[predicted_index], np.max(predictions[0])  # Return the emotion and its confidence score