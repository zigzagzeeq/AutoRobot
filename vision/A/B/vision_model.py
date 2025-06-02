from keras import Sequential
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D, MaxPooling2D
from keras.optimizers import Adam
import kagglehub

# Download latest version
path = kagglehub.dataset_download("msambare/fer2013")

print("Path to dataset files:", path)

emotion_model = Sequential()

emotion_model.add(Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=(48,48,1)))
emotion_model.add(Conv2D(64, kernel_size=(3, 3), activation='relu'))
emotion_model.add(MaxPooling2D(pool_size=(2, 2)))
emotion_model.add(Dropout(0.25))

emotion_model.add(Conv2D(128, kernel_size=(3, 3), activation='relu'))
emotion_model.add(MaxPooling2D(pool_size=(2, 2)))
emotion_model.add(Conv2D(128, kernel_size=(3, 3), activation='relu'))
emotion_model.add(MaxPooling2D(pool_size=(2, 2)))
emotion_model.add(Dropout(0.25))

emotion_model.add(Flatten())
emotion_model.add(Dense(1024, activation='relu'))
emotion_model.add(Dropout(0.5))
emotion_model.add(Dense(7, activation='softmax'))

import tensorflow as tf

# Define a learning rate schedule
learning_rate_schedule = tf.keras.optimizers.schedules.ExponentialDecay(
    initial_learning_rate=0.0001,
    decay_steps=10000,  # Adjust as needed
    decay_rate=0.9  # Adjust as needed
)

# Initialize the Adam optimizer with the learning rate schedule
optimizer = tf.keras.optimizers.Adam(learning_rate=learning_rate_schedule)

# Compile the model with the updated optimizer
emotion_model.compile(loss='categorical_crossentropy', optimizer=optimizer, metrics=['accuracy'])

emotion_model.load_weights('model.h5')
print(emotion_model.summary())
# Save the model
emotion_model.save('model_model.keras', save_format='keras')

# Save the model architecture
with open('model_architecture.json', 'w') as f:
    f.write(emotion_model.to_json())
# Save the model configuration
with open('model_config.json', 'w') as f:
    f.write(emotion_model.get_config())

# Save the model in TensorFlow SavedModel format
emotion_model.save('saved_model/my_model')
# Save the model in HDF5 format
emotion_model.save('model_model2.h5', save_format='h5')

# Save the model in TensorFlow Lite format
converter = tf.lite.TFLiteConverter.from_keras_model(emotion_model)
tflite_model = converter.convert()
with open('model.tflite', 'wb') as f:
    f.write(tflite_model)


# Save the model in ONNX format
import tf2onnx
onnx_model = tf2onnx.convert.from_keras(emotion_model)
with open('model.onnx', 'wb') as f:
    f.write(onnx_model.SerializeToString())
