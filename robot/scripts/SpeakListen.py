import io
import time
import base64
from gtts import gTTS
import speech_recognition as sr
from IPython.display import display, HTML
import pyttsx3

class SpeakListen:
    """Class to handle speaking and listening functionalities of the robot"""
    
    def __init__(self, robot):
        self.robot = robot

    # TODO Properly name the nodes and references
    def __init__(self, publisherNodeName, subcriberNodeName, topicName) -> None:
        self.engine = pyttsx3.init()
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        rospy.init_node(publisherNodeName)
        rospy.Subscriber(subcriberNodeName, Image, image_callback)
        self.pub = rospy.Publisher(topicName, String, queue_size=60)

    
    def speak(self, text):
        """Convert text to speech and play it"""
        print(f"🤖 Robot: {text}")
        # Queue the text for speaking
        try:
            self.engine.say(text)

            # Process and output the speech
            self.engine.runAndWait()
        except Exception as e:
            print(f"Error: {e}")

    def listen_for_speech(self, timeout=10):
            """Listen for user speech input"""
            print("🎤 Listening for your response...")
            try:
                with self.microphone as source:
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=10)
                
                text = self.recognizer.recognize_google(audio) 
                # TODO: Have issue because it provides error, you should test out at the bottom of the code
                print(f"👤 User said: {text}")
                return text
            
            except sr.WaitTimeoutError:
                print("⏰ No speech detected within timeout")
                return None
            except sr.UnknownValueError:
                print("❓ Could not understand audio")
                return None
            except sr.RequestError as e:
                print(f"🚫 Speech recognition error: {e}")
                return None
            
# sl = SpeakListen(robot=None)  # Replace None with actual robot instance if needed

# sl.speak("Hello! I am your robot assistant. How can I help you today?")
# print("Listening for your response...")
# print(sl.listen_for_speech(timeout=5))