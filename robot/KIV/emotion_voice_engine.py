import cv2
import numpy as np
import speech_recognition as sr
from gtts import gTTS
import io
import base64
from IPython.display import Audio, display, HTML, clear_output
import time
import random
from collections import Counter
import threading
import queue
import os
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
import requests
import zipfile

class EmotionVoiceEngine:
    def __init__(self):
        self.emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'neutral', 'sad', 'surprise']
        self.emotion_detector = None
        self.face_cascade = None
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.emotion_history = []
        self.is_running = False
        
    def setup_environment(self):
        """Install required packages and download models"""
        print("🤖 Setting up the robot environment...")
        
        # Install required packages
        os.system("pip install opencv-python gtts SpeechRecognition pyaudio")
        os.system("apt-get update && apt-get install -y portaudio19-dev python3-pyaudio")
        
        # Download emotion detection model
        try:
            print("📥 Downloading emotion detection model...")
            model_url = "https://github.com/oarriaga/face_classification/raw/master/trained_models/emotion_models/fer2013_mini_XCEPTION.102-0.66.hdf5"
            response = requests.get(model_url)
            with open('emotion_model.h5', 'wb') as f:
                f.write(response.content)
            print("✅ Emotion model downloaded successfully!")
        except:
            print("⚠️ Could not download emotion model. Using basic emotion detection.")
        
        # Initialize face cascade
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Load emotion detection model
        try:
            from tensorflow.keras.models import load_model
            self.emotion_detector = load_model('emotion_model.h5')
            print("✅ Emotion detection model loaded!")
        except:
            print("⚠️ Using basic emotion detection")
            self.emotion_detector = None
    
    def speak(self, text):
        """Convert text to speech and play it"""
        print(f"🤖 Robot: {text}")
        try:
            tts = gTTS(text=text, lang='en', slow=False)
            fp = io.BytesIO()
            tts.write_to_fp(fp)
            fp.seek(0)
            
            # Convert to base64 for HTML audio player
            audio_base64 = base64.b64encode(fp.read()).decode()
            audio_html = f"""
            <audio controls autoplay>
                <source src="data:audio/mp3;base64,{audio_base64}" type="audio/mp3">
            </audio>
            """
            display(HTML(audio_html))
            time.sleep(len(text.split()) * 0.5)  # Approximate speech duration
        except Exception as e:
            print(f"Speech error: {e}")
    
    def listen_for_speech(self, timeout=10):
        """Listen for user speech input"""
        print("🎤 Listening for your response...")
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            
            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=10)
            
            text = self.recognizer.recognize_google(audio)
            print(f"👤 User said: {text}")
            return text.lower()
        
        except sr.WaitTimeoutError:
            print("⏰ No speech detected within timeout")
            return None
        except sr.UnknownValueError:
            print("❓ Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"🚫 Speech recognition error: {e}")
            return None
    
    def detect_emotions_from_camera(self, duration=7):
        """Detect emotions from camera feed for specified duration"""
        print(f"📹 Starting emotion detection for {duration} seconds...")
        
        # For Colab, we'll simulate camera input since direct camera access is limited
        emotions_detected = []
        
        try:
            # In a real environment, this would capture from camera
            # For simulation, we'll generate some sample emotions
            print("📸 Simulating camera emotion detection...")
            
            # Simulate emotion detection over time
            possible_emotions = ['happy', 'sad', 'angry', 'surprise', 'fear', 'disgust', 'neutral']
            
            for i in range(duration):
                # Simulate detecting an emotion
                detected_emotion = random.choice(possible_emotions)
                emotions_detected.append(detected_emotion)
                print(f"  Frame {i+1}: Detected {detected_emotion}")
                time.sleep(1)
                
        except Exception as e:
            print(f"Camera error: {e}")
            # Fallback to simulated emotions
            emotions_detected = ['happy', 'happy', 'neutral', 'sad', 'happy']
        
        return emotions_detected
    
    def analyze_emotions(self, emotions_list):
        """Analyze detected emotions and determine dominant emotion"""
        if not emotions_list:
            return None, []
        
        # Count emotion frequencies
        emotion_counts = Counter(emotions_list)
        print(f"📊 Emotion frequencies: {dict(emotion_counts)}")
        
        # Find the highest frequency
        max_count = max(emotion_counts.values())
        tied_emotions = [emotion for emotion, count in emotion_counts.items() if count == max_count]
        
        # Apply tie-breaking rules
        if len(tied_emotions) > 1:
            # If neutral is in the tie, remove it unless it's the only emotion
            if 'neutral' in tied_emotions and len(tied_emotions) > 1:
                tied_emotions.remove('neutral')
            
            # Pick randomly from remaining tied emotions
            dominant_emotion = random.choice(tied_emotions)
            print(f"🎲 Tie detected! Randomly selected: {dominant_emotion}")
        else:
            dominant_emotion = tied_emotions[0]
        
        # If only neutral detected or no emotion, return None
        if dominant_emotion == 'neutral' and len(tied_emotions) == 1:
            return None, tied_emotions
        
        return dominant_emotion, tied_emotions
    
    def generate_emotion_response(self, emotion, tied_emotions=None):
        """Generate appropriate response based on detected emotion"""
        responses = {
            'happy': [
                "I can see you're feeling happy! That's wonderful to see.",
                "Your happiness is contagious! I love seeing that smile.",
                "You look joyful today! That brightens my circuits."
            ],
            'sad': [
                "I notice you seem sad. I'm here to listen and support you.",
                "You appear to be feeling down. Would you like to talk about it?",
                "I can sense some sadness. Remember, it's okay to feel this way."
            ],
            'angry': [
                "I can detect some anger. Let's take a moment to breathe together.",
                "You seem upset. I'm here to help you work through these feelings.",
                "I sense frustration. Sometimes talking helps release these emotions."
            ],
            'surprise': [
                "You look surprised! Something unexpected must have happened.",
                "I can see surprise in your expression. That must have caught you off guard!",
                "Your surprise is evident! Life can be full of unexpected moments."
            ],
            'fear': [
                "I detect some fear or anxiety. You're safe here with me.",
                "You seem worried or afraid. Let's work through this together.",
                "I sense some unease. Remember, I'm here to support you."
            ],
            'disgust': [
                "You appear disgusted or bothered by something. That must be unpleasant.",
                "I can see something is troubling you. Want to share what's bothering you?",
                "You seem put off by something. Sometimes talking helps."
            ]
        }
        
        if tied_emotions and len(tied_emotions) > 1:
            emotion_list = ", ".join(tied_emotions[:-1]) + f" and {tied_emotions[-1]}"
            return f"I'm detecting mixed emotions - {emotion_list}. You seem to be experiencing {emotion} most strongly."
        
        return random.choice(responses.get(emotion, ["I can sense your emotions. Let's talk about how you're feeling."]))
    
    def generate_llm_response(self, user_input, emotion):
        """Generate contextual response based on user input and emotion"""
        # Simple rule-based responses (in real scenario, you'd use an LLM API)
        suggestions = {
            'sad': [
                "It sounds like you're going through a difficult time. Remember that feelings are temporary and you're stronger than you know.",
                "I understand that can be really hard. Have you considered talking to someone you trust about this?",
                "That must be tough to deal with. Sometimes writing down your thoughts can help process these feelings."
            ],
            'angry': [
                "That sounds really frustrating. When we're angry, deep breathing can help calm our nervous system.",
                "I can understand why that would upset you. Physical exercise often helps release anger in a healthy way.",
                "It's natural to feel angry about that. Have you tried counting to ten or taking a short walk?"
            ],
            'happy': [
                "That's wonderful! It's great to hear about the positive things in your life.",
                "I'm so glad that brought you joy! Celebrating good moments is important for our wellbeing.",
                "That sounds amazing! Sharing happiness with others can multiply the good feelings."
            ],
            'fear': [
                "It's completely normal to feel afraid sometimes. Fear often means we care about something important.",
                "That does sound scary. Breaking big fears into smaller, manageable steps can help.",
                "Fear can be overwhelming. Grounding techniques like focusing on your five senses can help in the moment."
            ],
            'surprise': [
                "Life certainly has a way of surprising us! How are you processing this unexpected event?",
                "Surprises can be exciting or overwhelming. Take your time to understand how you feel about it.",
                "Unexpected moments often teach us something new about ourselves or our world."
            ]
        }
        
        base_response = random.choice(suggestions.get(emotion, [
            "Thank you for sharing that with me. Your feelings are valid and important.",
            "I appreciate you opening up about this. It takes courage to express our emotions.",
            "That gives me better insight into how you're feeling. You're not alone in this."
        ]))
        
        return base_response
    
    def suggest_activities(self, emotion):
        """Suggest appropriate activities based on emotion"""
        activities = {
            'sad': ['guided meditation', 'breathing exercise', 'gentle stretching', 'journaling prompts'],
            'angry': ['breathing exercise', 'progressive muscle relaxation', 'counting meditation', 'mindful walking'],
            'happy': ['gratitude exercise', 'energy boost activities', 'mindfulness practice', 'celebration ritual'],
            'fear': ['grounding exercise', 'breathing technique', 'calming meditation', 'anxiety relief'],
            'surprise': ['reflection exercise', 'mindfulness practice', 'processing technique', 'grounding exercise']
        }
        
        return activities.get(emotion, ['breathing exercise', 'mindfulness practice'])
    
    def demonstrate_activity(self, activity):
        """Demonstrate the selected activity"""
        if 'breathing' in activity:
            self.breathing_exercise()
        elif 'meditation' in activity:
            self.guided_meditation()
        elif 'stretching' in activity:
            self.gentle_stretching()
        elif 'grounding' in activity:
            self.grounding_exercise()
        else:
            self.default_relaxation()
    
    def breathing_exercise(self):
        """Guide user through breathing exercise"""
        self.speak("Let's do a simple breathing exercise together. Follow my guidance.")
        
        for i in range(3):
            self.speak(f"Round {i+1}. Breathe in slowly through your nose for 4 counts.")
            time.sleep(4)
            self.speak("Hold your breath for 4 counts.")
            time.sleep(4)
            self.speak("Now breathe out slowly through your mouth for 6 counts.")
            time.sleep(6)
            self.speak("Great job.")
            time.sleep(2)
        
        self.speak("Excellent! How do you feel now?")
    
    def guided_meditation(self):
        """Guide user through short meditation"""
        self.speak("Let's try a brief mindfulness meditation. Get comfortable and close your eyes if you'd like.")
        time.sleep(3)
        
        self.speak("Focus on your breathing. Notice the air coming in and going out.")
        time.sleep(5)
        
        self.speak("If your mind wanders, that's normal. Gently bring your attention back to your breath.")
        time.sleep(5)
        
        self.speak("Notice any sounds around you, but don't judge them. Just observe.")
        time.sleep(5)
        
        self.speak("Take one more deep breath, and when you're ready, open your eyes.")
        time.sleep(3)
        
        self.speak("Well done! Meditation takes practice, and you did great.")
    
    def grounding_exercise(self):
        """Guide 5-4-3-2-1 grounding technique"""
        self.speak("Let's try a grounding exercise called 5-4-3-2-1. This helps when feeling overwhelmed.")
        
        self.speak("Look around and name 5 things you can see.")
        time.sleep(8)
        
        self.speak("Now, name 4 things you can touch or feel.")
        time.sleep(6)
        
        self.speak("Name 3 things you can hear.")
        time.sleep(5)
        
        self.speak("Name 2 things you can smell.")
        time.sleep(4)
        
        self.speak("And finally, name 1 thing you can taste.")
        time.sleep(3)
        
        self.speak("Excellent! This technique helps bring you back to the present moment.")
    
    def gentle_stretching(self):
        """Guide through simple stretches"""
        self.speak("Let's do some gentle stretching to release tension.")
        
        self.speak("First, slowly roll your shoulders back 3 times.")
        time.sleep(5)
        
        self.speak("Now gently turn your head left and right, holding for 3 seconds each side.")
        time.sleep(8)
        
        self.speak("Reach your arms up high and take a deep breath in.")
        time.sleep(4)
        
        self.speak("Now slowly lower your arms and breathe out. Feel the tension releasing.")
        time.sleep(4)
        
        self.speak("Great job! Gentle movement can really help with emotional wellbeing.")
    
    def default_relaxation(self):
        """Default relaxation technique"""
        self.speak("Let's take a moment to relax together.")
        self.speak("Close your eyes and take three deep breaths with me.")
        
        for i in range(3):
            self.speak("Breathe in...")
            time.sleep(3)
            self.speak("And breathe out...")
            time.sleep(3)
        
        self.speak("You're doing great. Remember to be kind to yourself.")
    
    def main_loop(self):
        """Main program loop"""
        print("🤖 Robot activated! Starting main function...")
        self.speak("Hello! I'm your emotional support robot. I'm here to help you with your feelings.")
        
        self.is_running = True
        
        while self.is_running:
            try:
                # Step 3: Detect faces and emotions
                print("\n" + "="*50)
                print("🔍 Starting emotion detection...")
                
                emotions_detected = self.detect_emotions_from_camera(duration=7)
                
                if not emotions_detected:
                    self.speak("I didn't detect any faces. Let me try again.")
                    continue
                
                # Step 4-5: Analyze emotions and generate response
                dominant_emotion, tied_emotions = self.analyze_emotions(emotions_detected)
                
                if not dominant_emotion:
                    self.speak("I mostly detected neutral emotions. Let me continue monitoring.")
                    continue
                
                # Generate emotion-based response
                emotion_response = self.generate_emotion_response(dominant_emotion, tied_emotions)
                self.speak(emotion_response)
                
                # Step 6: Ask what made them feel this way
                self.speak("What made you feel this way today?")
                
                # Step 7: Get user response via speech recognition
                user_response = self.listen_for_speech(timeout=15)
                
                if not user_response:
                    self.speak("I didn't catch that. Let me check your emotions again.")
                    continue
                
                # Step 8: Generate LLM response
                llm_response = self.generate_llm_response(user_response, dominant_emotion)
                self.speak(llm_response)
                
                # Step 9: Ask about continuing with activities
                activities = self.suggest_activities(dominant_emotion)
                activity_list = ", ".join(activities[:-1]) + f", or {activities[-1]}" if len(activities) > 1 else activities[0]
                
                self.speak(f"Would you like me to guide you through a {activity_list} to help you feel better?")
                
                continue_response = self.listen_for_speech(timeout=10)
                
                if continue_response and any(word in continue_response for word in ['yes', 'sure', 'okay', 'please']):
                    # User wants to continue with activity
                    selected_activity = random.choice(activities)
                    self.speak(f"Great! Let's do a {selected_activity}.")
                    self.demonstrate_activity(selected_activity)
                    
                    # Ask if they want to continue or restart
                    self.speak("How was that? Would you like to continue our conversation or shall I monitor your emotions again?")
                    final_response = self.listen_for_speech(timeout=10)
                    
                    if final_response and any(word in final_response for word in ['continue', 'more', 'again']):
                        continue
                    elif final_response and any(word in final_response for word in ['monitor', 'check', 'emotions']):
                        continue
                    else:
                        self.speak("I'll continue monitoring your emotions. Take care!")
                        continue
                        
                else:
                    # User doesn't want activities, return to emotion detection
                    self.speak("No problem! I'll continue monitoring your emotions.")
                    continue
                    
            except KeyboardInterrupt:
                print("\n🛑 Robot shutting down...")
                self.speak("Goodbye! Take care of yourself!")
                self.is_running = False
                break
                
            except Exception as e:
                print(f"❌ Error in main loop: {e}")
                self.speak("I encountered an issue. Let me restart.")
                continue

# Initialize and run the emotion voice engine
def run_emotion_robot():
    """Main function to run the emotion detection robot"""
    robot = EmotionVoiceEngine()
    
    # Setup environment
    robot.setup_environment()
    
    print("\n" + "="*60)
    print("🤖 EMOTION DETECTION VOICE ENGINE")
    print("="*60)
    print("📋 Features:")
    print("  • Face and emotion detection")
    print("  • Speech recognition and synthesis")
    print("  • Emotional support responses")
    print("  • Guided activities (breathing, meditation, etc.)")
    print("  • Continuous monitoring loop")
    print("\n🎯 Instructions:")
    print("  • Make sure your camera and microphone are working")
    print("  • Speak clearly when prompted")
    print("  • Press Ctrl+C to stop the robot")
    print("="*60)
    
    # Start the main program
    try:
        robot.main_loop()
    except Exception as e:
        print(f"🚫 Critical error: {e}")
        print("🔧 Try restarting the robot or check your hardware connections.")

# Run the robot
if __name__ == "__main__":
    run_emotion_robot()