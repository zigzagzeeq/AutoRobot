# AutoRobot

## 1. System Architecture Overview

### 1.1 Core Components
- **Speech-to-Text (STT) Module**: Converts user voice input to text
- **Text-to-Speech (TTS) Module**: Synthesizes robot responses into speech
- **Rule-Based Response Engine**: Generates appropriate responses based on emotion and context
- **ROS Integration Layer**: Interfaces with other system components via ROS topics

### 1.2 Technology Stack
- **STT Framework**: Google Speech Recognition API / SpeechRecognition library
- **TTS Framework**: pyttsx3 / Google Text-to-Speech
- **Audio Processing**: pyaudio for microphone input/output
- **ROS Integration**: rospy for node communication
- **Language Processing**: NLTK/spaCy for text preprocessing

## 2. Implementation Details

### 2.1 Speech-to-Text Implementation

#### 2.1.1 Audio Capture System
```python
# Microphone configuration
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
RECORD_SECONDS = 5
```

#### 2.1.2 STT Processing Pipeline
1. **Audio Capture**: Continuous listening with voice activity detection
2. **Noise Filtering**: Background noise reduction and audio enhancement
3. **Speech Recognition**: Real-time conversion using Google Speech API
4. **Text Processing**: Cleaning and normalization of recognized text

#### 2.1.3 Key Features
- **Wake Word Detection**: "Hey Robot" activation phrase
- **Continuous Listening**: Background audio monitoring
- **Error Handling**: Fallback mechanisms for recognition failures
- **Multi-language Support**: English primary, with extensibility

### 2.2 Text-to-Speech Implementation

#### 2.2.1 Voice Synthesis Configuration
```python
# TTS Engine Setup
engine = pyttsx3.init()
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[0].id)  # Female voice
engine.setProperty('rate', 150)  # Speech rate
engine.setProperty('volume', 0.8)  # Volume level
```

#### 2.2.2 Emotional Voice Modulation
- **Happy Response**: Higher pitch, faster rate
- **Sad Response**: Lower pitch, slower rate
- **Angry Response**: Firm tone, moderate pace
- **Neutral Response**: Standard voice parameters

### 2.3 Rule-Based Response Engine

#### 2.3.1 Response Generation Framework
```python
class ResponseEngine:
    def __init__(self):
        self.emotion_responses = {
            'happy': self.load_happy_responses(),
            'sad': self.load_sad_responses(),
            'angry': self.load_angry_responses(),
            'neutral': self.load_neutral_responses(),
            'fear': self.load_fear_responses(),
            'surprise': self.load_surprise_responses()
        }
```

#### 2.3.2 Context-Aware Response Selection
- **Emotion-Based Responses**: Tailored to detected facial emotion
- **Intent Recognition**: Basic keyword matching for user requests
- **Conversation Memory**: Short-term context retention
- **Fallback Responses**: Generic responses for unknown inputs

#### 2.3.3 Response Categories
1. **Empathetic Responses**: For emotional support
2. **Informational Responses**: For questions and requests
3. **Conversational Responses**: For casual interaction
4. **Encouragement Responses**: For positive reinforcement

### 2.4 ROS Integration

#### 2.4.1 ROS Node Structure
```python
class SpeechDialogueNode:
    def __init__(self):
        rospy.init_node('speech_dialogue_node')
        
        # Subscribers
        self.emotion_sub = rospy.Subscriber('/emotion_detection', String, self.emotion_callback)
        
        # Publishers
        self.response_pub = rospy.Publisher('/robot_response', String, queue_size=10)
        self.audio_pub = rospy.Publisher('/audio_output', String, queue_size=10)
```

#### 2.4.2 Topic Communications
- **Input Topics**:
  - `/emotion_detection`: Receives emotion data from vision system
  - `/user_speech`: Processes recognized speech text
- **Output Topics**:
  - `/robot_response`: Publishes generated text responses
  - `/audio_output`: Controls TTS audio playback

### Test STT + TTS options.

```python
#!/usr/bin/env python3
"""
STT + TTS Pipeline Implementation with Testing
Speech-to-Text → Rule-based Dialogue → Text-to-Speech
Author: Dexter - Speech & Dialogue Engineer
"""

import speech_recognition as sr
import pyttsx3
import pyaudio
import wave
import threading
import time
import json
import re
from datetime import datetime
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class STTEngine:
    """Speech-to-Text Engine with multiple testing options"""
    
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.is_listening = False
        
        # Calibrate for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            logger.info("STT Engine initialized and calibrated for ambient noise")
    
    def listen_once(self, timeout=5):
        """Single speech recognition attempt"""
        try:
            with self.microphone as source:
                logger.info("Listening for speech...")
                audio = self.recognizer.listen(source, timeout=timeout)
                
            logger.info("Processing speech...")
            text = self.recognizer.recognize_google(audio)
            logger.info(f"Recognized: {text}")
            return text
            
        except sr.WaitTimeoutError:
            logger.warning("Listening timeout - no speech detected")
            return None
        except sr.UnknownValueError:
            logger.warning("Could not understand audio")
            return None
        except sr.RequestError as e:
            logger.error(f"Speech recognition error: {e}")
            return None
    
    def listen_continuous(self, callback):
        """Continuous listening with callback"""
        self.is_listening = True
        
        def listen_loop():
            while self.is_listening:
                text = self.listen_once(timeout=2)
                if text:
                    callback(text)
                time.sleep(0.1)
        
        thread = threading.Thread(target=listen_loop)
        thread.daemon = True
        thread.start()
        logger.info("Started continuous listening")
        return thread
    
    def stop_listening(self):
        """Stop continuous listening"""
        self.is_listening = False
        logger.info("Stopped continuous listening")
    
    def test_microphone(self):
        """Test microphone functionality"""
        try:
            with self.microphone as source:
                logger.info("Testing microphone... Say something!")
                audio = self.recognizer.listen(source, timeout=3)
                text = self.recognizer.recognize_google(audio)
                logger.info(f"Microphone test successful: {text}")
                return True, text
        except Exception as e:
            logger.error(f"Microphone test failed: {e}")
            return False, str(e)

class RuleBasedDialogue:
    """Rule-based dialogue system with emotion context"""
    
    def __init__(self):
        self.conversation_history = []
        self.current_emotion = "neutral"
        self.user_name = None
        
        # Load response templates
        self.responses = self._load_responses()
        logger.info("Rule-based dialogue system initialized")
    
    def _load_responses(self):
        """Load response templates for different emotions and intents"""
        return {
            "greetings": {
                "happy": [
                    "Hello! You seem cheerful today! How can I help you?",
                    "Hi there! Your smile is contagious! What's on your mind?",
                    "Hey! You look fantastic! What can I do for you?"
                ],
                "sad": [
                    "Hello... I can see you might be feeling down. I'm here to listen.",
                    "Hi there. Is everything okay? I'm here if you need to talk.",
                    "Hey... You seem a bit blue. Would you like to share what's bothering you?"
                ],
                "angry": [
                    "Hello. I can sense some tension. Let's talk about what's bothering you.",
                    "Hi. I'm here to help work through whatever is frustrating you.",
                    "Hey there. Take a deep breath. I'm here to listen."
                ],
                "neutral": [
                    "Hello! How can I assist you today?",
                    "Hi there! What can I do for you?",
                    "Hey! How are you doing today?"
                ]
            },
            "questions": {
                "happy": [
                    "That's a great question! Let me think about that for you.",
                    "I love your curiosity! Here's what I know about that.",
                    "Excellent question! I'm excited to help you with that."
                ],
                "sad": [
                    "I understand you're looking for answers. Let me help you with that.",
                    "That's an important question. I'll do my best to help.",
                    "I can see this matters to you. Here's what I can tell you."
                ],
                "neutral": [
                    "That's a good question. Let me help you with that.",
                    "I can help you with that. Here's what I know.",
                    "Let me provide you with some information about that."
                ]
            },
            "compliments": {
                "happy": [
                    "Thank you so much! Your kindness makes me happy too!",
                    "That's so sweet of you to say! You're wonderful!",
                    "Aww, thank you! You just made my day brighter!"
                ],
                "neutral": [
                    "Thank you, that's very kind of you to say!",
                    "I appreciate your kind words!",
                    "That's really nice, thank you!"
                ]
            },
            "farewells": {
                "happy": [
                    "Goodbye! It was wonderful talking with you! Have an amazing day!",
                    "See you later! Keep that beautiful smile!",
                    "Bye! Thanks for the lovely conversation!"
                ],
                "sad": [
                    "Take care of yourself. Remember, tomorrow is a new day.",
                    "Goodbye. I hope things get better for you soon.",
                    "See you later. You're stronger than you know."
                ],
                "neutral": [
                    "Goodbye! Have a great day!",
                    "See you later! Take care!",
                    "Bye! It was nice talking with you!"
                ]
            },
            "default": {
                "happy": [
                    "I can see you're in a great mood! Tell me more about that!",
                    "Your positive energy is wonderful! What else is on your mind?",
                    "I love your enthusiasm! How can I help you further?"
                ],
                "sad": [
                    "I'm here to listen. Sometimes talking helps.",
                    "It's okay to feel this way. Would you like to share more?",
                    "I understand. Take your time, I'm here for you."
                ],
                "angry": [
                    "I can sense your frustration. Let's work through this together.",
                    "It's natural to feel upset sometimes. I'm here to help.",
                    "Take a moment to breathe. I'm listening."
                ],
                "neutral": [
                    "I see. Can you tell me more about that?",
                    "That's interesting. What else would you like to discuss?",
                    "I understand. How can I help you with that?"
                ]
            }
        }
    
    def set_emotion(self, emotion):
        """Set current detected emotion"""
        self.current_emotion = emotion.lower()
        logger.info(f"Emotion context set to: {self.current_emotion}")
    
    def classify_intent(self, text):
        """Simple rule-based intent classification"""
        text_lower = text.lower()
        
        # Greeting patterns
        greetings = ["hello", "hi", "hey", "good morning", "good afternoon", "good evening"]
        if any(greeting in text_lower for greeting in greetings):
            return "greetings"
        
        # Question patterns
        question_words = ["what", "how", "why", "when", "where", "who", "can you", "do you", "?"]
        if any(word in text_lower for word in question_words):
            return "questions"
        
        # Compliment patterns
        compliments = ["thank you", "thanks", "you're great", "good job", "well done", "awesome"]
        if any(comp in text_lower for comp in compliments):
            return "compliments"
        
        # Farewell patterns
        farewells = ["goodbye", "bye", "see you", "farewell", "talk to you later", "gtg"]
        if any(farewell in text_lower for farewell in farewells):
            return "farewells"
        
        return "default"
    
    def generate_response(self, user_input):
        """Generate contextual response based on input and emotion"""
        intent = self.classify_intent(user_input)
        
        # Get appropriate responses for current emotion and intent
        if intent in self.responses and self.current_emotion in self.responses[intent]:
            responses = self.responses[intent][self.current_emotion]
        elif intent in self.responses and "neutral" in self.responses[intent]:
            responses = self.responses[intent]["neutral"]
        else:
            responses = self.responses["default"][self.current_emotion]
        
        # Select response (could be random, but using first for consistency in testing)
        import random
        response = random.choice(responses)
        
        # Log conversation
        self.conversation_history.append({
            "timestamp": datetime.now().isoformat(),
            "user_input": user_input,
            "intent": intent,
            "emotion": self.current_emotion,
            "response": response
        })
        
        logger.info(f"Intent: {intent}, Emotion: {self.current_emotion}")
        logger.info(f"Response: {response}")
        
        return response
    
    def get_conversation_history(self):
        """Get conversation history"""
        return self.conversation_history

class TTSEngine:
    """Text-to-Speech Engine with emotion-based voice modulation"""
    
    def __init__(self):
        self.engine = pyttsx3.init()
        self.voices = self.engine.getProperty('voices')
        
        # Default voice settings
        self.default_settings = {
            'rate': 150,
            'volume': 0.8,
            'voice': 0  # Female voice index
        }
        
        # Emotion-based voice settings
        self.emotion_settings = {
            'happy': {'rate': 170, 'volume': 0.9, 'voice': 0},
            'sad': {'rate': 120, 'volume': 0.6, 'voice': 0},
            'angry': {'rate': 160, 'volume': 0.8, 'voice': 1},  # Male voice for angry
            'neutral': {'rate': 150, 'volume': 0.8, 'voice': 0}
        }
        
        self._apply_settings(self.default_settings)
        logger.info("TTS Engine initialized")
    
    def _apply_settings(self, settings):
        """Apply voice settings to TTS engine"""
        self.engine.setProperty('rate', settings['rate'])
        self.engine.setProperty('volume', settings['volume'])
        if self.voices and len(self.voices) > settings['voice']:
            self.engine.setProperty('voice', self.voices[settings['voice']].id)
    
    def speak(self, text, emotion="neutral"):
        """Convert text to speech with emotion-based modulation"""
        try:
            # Apply emotion-based settings
            if emotion.lower() in self.emotion_settings:
                settings = self.emotion_settings[emotion.lower()]
                self._apply_settings(settings)
                logger.info(f"Applied {emotion} voice settings")
            
            logger.info(f"Speaking: {text}")
            self.engine.say(text)
            self.engine.runAndWait()
            
            # Reset to default settings
            self._apply_settings(self.default_settings)
            
        except Exception as e:
            logger.error(f"TTS Error: {e}")
    
    def test_voice(self):
        """Test TTS functionality"""
        test_phrases = [
            ("Hello! This is a test of the text-to-speech system.", "neutral"),
            ("I'm so excited! This sounds amazing!", "happy"),
            ("I'm feeling a bit down today...", "sad"),
            ("I'm quite frustrated with this situation!", "angry")
        ]
        
        for phrase, emotion in test_phrases:
            print(f"Testing {emotion} voice...")
            self.speak(phrase, emotion)
            time.sleep(1)

class SpeechDialoguePipeline:
    """Complete STT → Dialogue → TTS Pipeline"""
    
    def __init__(self):
        self.stt = STTEngine()
        self.dialogue = RuleBasedDialogue()
        self.tts = TTSEngine()
        self.is_running = False
        
        logger.info("Speech Dialogue Pipeline initialized")
    
    def process_single_interaction(self, emotion="neutral"):
        """Process a single speech interaction"""
        # Set emotion context
        self.dialogue.set_emotion(emotion)
        
        # Step 1: Speech-to-Text
        print(f"\n[Emotion Context: {emotion}]")
        print("Listening for your input...")
        user_input = self.stt.listen_once(timeout=10)
        
        if not user_input:
            print("No speech detected or recognition failed.")
            return None
        
        print(f"You said: {user_input}")
        
        # Step 2: Generate response
        response = self.dialogue.generate_response(user_input)
        print(f"Robot response: {response}")
        
        # Step 3: Text-to-Speech
        self.tts.speak(response, emotion)
        
        return {
            'user_input': user_input,
            'response': response,
            'emotion': emotion
        }
    
    def run_continuous_mode(self):
        """Run continuous interaction mode"""
        self.is_running = True
        print("\n=== Continuous Speech Dialogue Mode ===")
        print("Say 'stop listening' to exit")
        
        def process_speech(text):
            if "stop listening" in text.lower():
                self.stop()
                return
            
            response = self.dialogue.generate_response(text)
            print(f"\nYou: {text}")
            print(f"Robot: {response}")
            self.tts.speak(response, self.dialogue.current_emotion)
        
        self.stt.listen_continuous(process_speech)
        
        try:
            while self.is_running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()
    
    def stop(self):
        """Stop the pipeline"""
        self.is_running = False
        self.stt.stop_listening()
        print("\nSpeech dialogue pipeline stopped.")
    
    def run_test_suite(self):
        """Run comprehensive test suite"""
        print("\n" + "="*50)
        print("SPEECH DIALOGUE PIPELINE TEST SUITE")
        print("="*50)
        
        # Test 1: Component Tests
        print("\n1. TESTING INDIVIDUAL COMPONENTS")
        print("-" * 30)
        
        # Test STT
        print("Testing STT (Microphone)...")
        success, result = self.stt.test_microphone()
        print(f"STT Test: {'PASSED' if success else 'FAILED'} - {result}")
        
        # Test TTS
        print("\nTesting TTS (Voice Output)...")
        self.tts.test_voice()
        print("TTS Test: COMPLETED")
        
        # Test Dialogue System
        print("\nTesting Rule-based Dialogue...")
        test_inputs = [
            "Hello there!",
            "How are you doing?",
            "Thank you for your help",
            "Goodbye!"
        ]
        
        for test_input in test_inputs:
            response = self.dialogue.generate_response(test_input)
            print(f"Input: '{test_input}' → Response: '{response}'")
        
        # Test 2: Emotion Context Tests
        print("\n2. TESTING EMOTION CONTEXTS")
        print("-" * 30)
        
        emotions = ["happy", "sad", "angry", "neutral"]
        for emotion in emotions:
            print(f"\nTesting {emotion} context:")
            self.dialogue.set_emotion(emotion)
            response = self.dialogue.generate_response("Hello")
            print(f"Response: {response}")
            self.tts.speak(f"This is {emotion} voice", emotion)
        
        # Test 3: Integration Test
        print("\n3. INTEGRATION TEST")
        print("-" * 30)
        print("Testing complete STT → Dialogue → TTS pipeline...")
        
        result = self.process_single_interaction("neutral")
        if result:
            print("Integration Test: PASSED")
        else:
            print("Integration Test: FAILED")
        
        print("\n" + "="*50)
        print("TEST SUITE COMPLETED")
        print("="*50)

def main():
    """Main function with menu system"""
    pipeline = SpeechDialoguePipeline()
    
    while True:
        print("\n" + "="*40)
        print("SPEECH DIALOGUE PIPELINE TESTER")
        print("="*40)
        print("1. Run Test Suite")
        print("2. Single Interaction Test")
        print("3. Continuous Mode")
        print("4. Component Tests Only")
        print("5. Exit")
        print("-" * 40)
        
        choice = input("Select option (1-5): ").strip()
        
        if choice == "1":
            pipeline.run_test_suite()
        
        elif choice == "2":
            emotions = ["neutral", "happy", "sad", "angry"]
            print("\nAvailable emotions:", emotions)
            emotion = input("Enter emotion context (default: neutral): ").strip()
            if emotion not in emotions:
                emotion = "neutral"
            
            result = pipeline.process_single_interaction(emotion)
            if result:
                print(f"\nInteraction completed successfully!")
                print(f"Emotion context: {result['emotion']}")
        
        elif choice == "3":
            emotion = input("Enter emotion context for session (default: neutral): ").strip()
            if not emotion:
                emotion = "neutral"
            pipeline.dialogue.set_emotion(emotion)
            pipeline.run_continuous_mode()
        
        elif choice == "4":
            print("\n--- Component Tests ---")
            
            # STT Test
            print("1. Testing STT...")
            success, result = pipeline.stt.test_microphone()
            print(f"Result: {result}")
            
            # TTS Test
            print("\n2. Testing TTS...")
            pipeline.tts.test_voice()
            
            # Dialogue Test
            print("\n3. Testing Dialogue System...")
            test_response = pipeline.dialogue.generate_response("Hello!")
            print(f"Sample response: {test_response}")
        
        elif choice == "5":
            print("Goodbye!")
            break
        
        else:
            print("Invalid option. Please try again.")

if __name__ == "__main__":
    # Required packages installation check
    required_packages = ["speechrecognition", "pyttsx3", "pyaudio"]
    
    print("Speech Dialogue Pipeline - STT + TTS Implementation")
    print("=" * 50)
    print("\nRequired packages:", ", ".join(required_packages))
    print("\nTo install required packages:")
    print("pip install speechrecognition pyttsx3 pyaudio")
    print("\nNote: pyaudio might require additional system dependencies")
    print("Ubuntu/Debian: sudo apt-get install portaudio19-dev")
    print("macOS: brew install portaudio")
    print("Windows: Usually works with pip install")
    print("\n" + "=" * 50)
    
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        print(f"Error: {e}")

```

### ROS Node Implementation for STT + TTS Pipeline
```python
#!/usr/bin/env python3
"""
ROS Node Implementation for STT + TTS Pipeline
Integrates with emotion detection and robot control system
Author: Dexter - Speech & Dialogue Engineer
"""

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import speech_recognition as sr
import pyttsx3
import threading
import json
from datetime import datetime
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SpeechDialogueROSNode:
    """ROS Node for Speech Dialogue System"""
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('speech_dialogue_node', anonymous=True)
        
        # Initialize speech components
        self.stt = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.tts = pyttsx3.init()
        
        # ROS Publishers
        self.response_pub = rospy.Publisher('/robot_response', String, queue_size=10)
        self.speech_status_pub = rospy.Publisher('/speech_status', String, queue_size=10)
        self.dialogue_log_pub = rospy.Publisher('/dialogue_log', String, queue_size=10)
        
        # ROS Subscribers
        self.emotion_sub = rospy.Subscriber('/emotion_detection', String, self.emotion_callback)
        self.wake_word_sub = rospy.Subscriber('/wake_word_detected', Bool, self.wake_word_callback)
        self.system_control_sub = rospy.Subscriber('/system_control', String, self.system_control_callback)
        
        # State variables
        self.current_emotion = "neutral"
        self.is_listening = False
        self.is_speaking = False
        self.conversation_active = False
        
        # Initialize TTS settings
        self.setup_tts()
        
        # Calibrate microphone
        self.calibrate_microphone()
        
        # Response templates
        self.load_response_templates()
        
        rospy.loginfo("Speech Dialogue ROS Node initialized")
        
        # Start listening thread
        self.listening_thread = threading.Thread(target=self.continuous_listening_loop)
        self.listening_thread.daemon = True
        self.listening_thread.start()
    
    def setup_tts(self):
        """Configure TTS engine with emotion-based settings"""
        voices = self.tts.getProperty('voices')
        
        # Emotion-based voice configurations
        self.voice_configs = {
            'happy': {'rate': 170, 'volume': 0.9, 'voice': 0},
            'sad': {'rate': 120, 'volume': 0.6, 'voice': 0},
            'angry': {'rate': 160, 'volume': 0.8, 'voice': 1 if len(voices) > 1 else 0},
            'fear': {'rate': 140, 'volume': 0.7, 'voice': 0},
            'surprise': {'rate': 180, 'volume': 0.9, 'voice': 0},
            'neutral': {'rate': 150, 'volume': 0.8, 'voice': 0}
        }
        
        # Set default configuration
        self.apply_voice_config('neutral')
    
    def apply_voice_config(self, emotion):
        """Apply emotion-specific voice configuration"""
        if emotion in self.voice_configs:
            config = self.voice_configs[emotion]
            self.tts.setProperty('rate', config['rate'])
            self.tts.setProperty('volume', config['volume'])
            
            voices = self.tts.getProperty('voices')
            if voices and len(voices) > config['voice']:
                self.tts.setProperty('voice', voices[config['voice']].id)
    
    def calibrate_microphone(self):
        """Calibrate microphone for ambient noise"""
        try:
            with self.microphone as source:
                self.stt.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Microphone calibrated for ambient noise")
        except Exception as e:
            rospy.logerr(f"Microphone calibration failed: {e}")
    
    def load_response_templates(self):
        """Load response templates for different contexts"""
        self.responses = {
            "greetings": {
                "happy": [
                    "Hello! You look wonderful today! How can I brighten your day even more?",
                    "Hi there! Your joy is contagious! What can I help you with?",
                    "Hey! I love seeing you so happy! What's on your mind?"
                ],
                "sad": [
                    "Hello... I can see you're not feeling your best. I'm here for you.",
                    "Hi there. Would you like to talk about what's bothering you?",
                    "Hey... Sometimes a friendly chat can help. I'm listening."
                ],
                "angry": [
                    "Hello. I sense some frustration. Let's work through this together.",
                    "Hi. Take a deep breath. I'm here to help you feel better.",
                    "Hey there. It's okay to feel upset. Let's talk about it."
                ],
                "fear": [
                    "Hello. You seem worried. Don't worry, I'm here to help you feel safe.",
                    "Hi there. Whatever is frightening you, we can face it together.",
                    "Hey. It's natural to feel scared sometimes. You're not alone."
                ],
                "surprise": [
                    "Hello! You look amazed! What's got you so surprised?",
                    "Hi there! Something exciting must have happened! Tell me about it!",
                    "Hey! Your surprise is written all over your face! What's going on?"
                ],
                "neutral": [
                    "Hello! How can I assist you today?",
                    "Hi there! What can I do for you?",
                    "Hey! How are things going?"
                ]
            },
            "questions": {
                "happy": [
                    "That's an exciting question! I'm thrilled to help you with that!",
                    "Great question! Your curiosity is wonderful!",
                    "I love that you asked! Here's what I can tell you..."
                ],
                "sad": [
                    "That's an important question. I hope my answer helps you feel better.",
                    "I understand this matters to you. Let me help as best I can.",
                    "That's a thoughtful question. Here's what I know..."
                ],
                "neutral": [
                    "That's a good question. Let me help you with that.",
                    "I can assist you with that. Here's what I know...",
                    "Interesting question! Let me provide some information..."
                ]
            },
            "farewells": {
                "happy": [
                    "Goodbye! Keep that beautiful smile! Have an amazing day!",
                    "See you later! Your happiness made my day too!",
                    "Bye! It was wonderful talking with such a joyful person!"
                ],
                "sad": [
                    "Take care. Remember, brighter days are ahead.",
                    "Goodbye. I hope you feel better soon. You're stronger than you know.",
                    "See you later. Don't forget that you matter."
                ],
                "neutral": [
                    "Goodbye! Have a great day!",
                    "See you later! Take care!",
                    "Bye! It was nice talking with you!"
                ]
            },
            "default": {
                "happy": [
                    "I can see you're in great spirits! Tell me more!",
                    "Your positive energy is amazing! What else is on your mind?",
                    "I love your enthusiasm! How can I help you further?"
                ],
                "sad": [
                    "I'm here to listen. Take your time.",
                    "It's okay to feel this way. Would you like to share more?",
                    "I understand. Sometimes talking helps."
                ],
                "angry": [
                    "I can sense your frustration. Let's work through this.",
                    "It's natural to feel upset. I'm here to help.",
                    "Take a moment to breathe. I'm listening."
                ],
                "fear": [
                    "You seem worried. I'm here to help you feel safer.",
                    "It's okay to be scared. We can talk through your concerns.",
                    "Don't worry. I'm here to support you."
                ],
                "surprise": [
                    "You seem amazed! What's got you so surprised?",
                    "Something exciting must be happening! Tell me about it!",
                    "Your surprise is evident! What's going on?"
                ],
                "neutral": [
                    "I see. Can you tell me more about that?",
                    "That's interesting. What else would you like to discuss?",
                    "I understand. How can I help you with that?"
                ]
            }
        }
    
    def emotion_callback(self, msg):
        """Handle emotion detection updates"""
        try:
            emotion_data = json.loads(msg.data)
            self.current_emotion = emotion_data.get('dominant_emotion', 'neutral').lower()
            confidence = emotion_data.get('confidence', 0.0)
            
            rospy.loginfo(f"Emotion updated: {self.current_emotion} (confidence: {confidence:.2f})")
            
            # Publish emotion status
            status_msg = String()
            status_msg.data = f"emotion_context_updated:{self.current_emotion}:{confidence}"
            self.speech_status_pub.publish(status_msg)
            
        except json.JSONDecodeError:
            # Handle simple string emotions
            self.current_emotion = msg.data.lower()
            rospy.loginfo(f"Emotion updated: {self.current_emotion}")
    
    def wake_word_callback(self, msg):
        """Handle wake word detection"""
        if msg.data:
            rospy.loginfo("Wake word detected - activating conversation")
            self.conversation_active = True
            
            # Acknowledge wake word
            self.speak_response("Yes? How can I help you?", self.current_emotion)
            
            # Publish status
            status_msg = String()
            status_msg.data = "wake_word_detected:conversation_activated"
            self.speech_status_pub.publish(status_msg)
    
    def system_control_callback(self, msg):
        """Handle system control commands"""
        command = msg.data.lower()
        
        if command == "start_listening":
            self.conversation_active = True
            rospy.loginfo("Speech dialogue activated via system control")
        elif command == "stop_listening":
            self.conversation_active = False
            rospy.loginfo("Speech dialogue deactivated via system control")
        elif command == "calibrate_microphone":
            self.calibrate_microphone()
        elif command == "test_tts":
            self.test_tts_system()
    
    def continuous_listening_loop(self):
        """Main listening loop running in separate thread"""
        while not rospy.is_shutdown():
            if self.conversation_active and not self.is_speaking:
                try:
                    self.listen_for_speech()
                except Exception as e:
                    rospy.logerr(f"Listening error: {e}")
            
            rospy.sleep(0.1)  # Small delay to prevent CPU overload
    
    def listen_for_speech(self):
        """Listen for speech input and process"""
        try:
            with self.microphone as source:
                # Listen with timeout
                audio = self.stt.listen(source, timeout=2, phrase_time_limit=5)
            
            # Publish listening status
            status_msg = String()
            status_msg.data = "processing_speech"
            self.speech_status_pub.publish(status_msg)
            
            # Recognize speech
            text = self.stt.recognize_google(audio)
            rospy.loginfo(f"Speech recognized: {text}")
            
            # Process the recognized text
            self.process_user_input(text)
            
        except sr.WaitTimeoutError:
            # Timeout is normal, just continue listening
            pass
        except sr.UnknownValueError:
            rospy.logwarn("Could not understand audio")
        except sr.RequestError as e:
            rospy.logerr(f"Speech recognition error: {e}")
    
    def process_user_input(self, user_input):
        """Process user input and generate response"""
        # Log the interaction
        interaction_data = {
            'timestamp': datetime.now().isoformat(),
            'user_input': user_input,
            'emotion_context': self.current_emotion
        }
        
        # Check for conversation end commands
        if any(phrase in user_input.lower() for phrase in ["goodbye", "bye", "stop talking", "that's all"]):
            self.conversation_active = False
            response = self.generate_response(user_input, "farewells")
            self.speak_response(response, self.current_emotion)
            
            # Publish end of conversation
            status_msg = String()
            status_msg.data = "conversation_ended"
            self.speech_status_pub.publish(status_msg)
            return
        
        # Generate and speak response
        response = self.generate_response(user_input)
        self.speak_response(response, self.current_emotion)
        
        # Log complete interaction
        interaction_data['response'] = response
        log_msg = String()
        log_msg.data = json.dumps(interaction_data)
        self.dialogue_log_pub.publish(log_msg)
        
        # Publish user input for other nodes
        response_msg = String()
        response_msg.data = json.dumps({
            'user_input': user_input,
            'robot_response': response,
            'emotion_context': self.current_emotion
        })
        self.response_pub.publish(response_msg)
    
    def classify_intent(self, text):
        """Classify user intent from text"""
        text_lower = text.lower()
        
        # Greeting patterns
        greetings = ["hello", "hi", "hey", "good morning", "good afternoon", "good evening"]
        if any(greeting in text_lower for greeting in greetings):
            return "greetings"
        
        # Question patterns
        question_words = ["what", "how", "why", "when", "where", "who", "can you", "do you", "?"]
        if any(word in text_lower for word in question_words):
            return "questions"
        
        # Farewell patterns
        farewells = ["goodbye", "bye", "see you", "farewell", "talk to you later"]
        if any(farewell in text_lower for farewell in farewells):
            return "farewells"
        
        return "default"
    
    def generate_response(self, user_input, intent_override=None):
        """Generate contextual response based on input and emotion"""
        intent = intent_override or self.classify_intent(user_input)
        
        # Get appropriate responses for current emotion and intent
        if intent in self.responses and self.current_emotion in self.responses[intent]:
            responses = self.responses[intent][self.current_emotion]
        elif intent in self.responses and "neutral" in self.responses[intent]:
            responses = self.responses[intent]["neutral"]
        else:
            responses = self.responses["default"].get(self.current_emotion, 
                       self.responses["default"]["neutral"])
        
        # Select response (rotate through responses for variety)
        import random
        response = random.choice(responses)
        
        rospy.loginfo(f"Generated response for intent '{intent}' with emotion '{self.current_emotion}': {response}")
        return response
    
    def speak_response(self, text, emotion="neutral"):
        """Convert text to speech with emotion-based voice modulation"""
        if self.is_speaking:
            return  # Prevent overlapping speech
        
        self.is_speaking = True
        
        try:
            # Apply emotion-specific voice settings
            self.apply_voice_config(emotion)
            
            # Publish speaking status
            status_msg = String()
            status_msg.data = f"speaking:{emotion}:{text[:50]}..."
            self.speech_status_pub.publish(status_msg)
            
            rospy.loginfo(f"Speaking with {emotion} voice: {text}")
            
            # Speak the text
            self.tts.say(text)
            self.tts.runAndWait()
            
        except Exception as e:
            rospy.logerr(f"TTS error: {e}")
        finally:
            self.is_speaking = False
            
            # Publish speaking complete status
            status_msg = String()
            status_msg.data = "speaking_complete"
            self.speech_status_pub.publish(status_msg)
    
    def test_tts_system(self):
        """Test TTS system with different emotions"""
        test_phrases = [
            ("Testing neutral voice", "neutral"),
            ("I'm so happy and excited!", "happy"),
            ("I'm feeling quite sad today", "sad"),
            ("I'm really angry about this!", "angry"),
            ("This is so surprising and amazing!", "surprise"),
            ("I'm a bit scared and worried", "fear")
        ]
        
        rospy.loginfo("Starting TTS system test")
        
        for phrase, emotion in test_phrases:
            rospy.loginfo(f"Testing {emotion} voice...")
            self.speak_response(phrase, emotion)
            rospy.sleep(1)  # Brief pause between tests
        
        rospy.loginfo("TTS system test completed")

def test_stt_tts_integration():
    """Standalone test function for STT + TTS integration"""
    print("=" * 50)
    print("STT + TTS INTEGRATION TEST")
    print("=" * 50)
    
    # Initialize components
    stt = sr.Recognizer()
    microphone = sr.Microphone()
    tts = pyttsx3.init()
    
    # Calibrate microphone
    print("Calibrating microphone for ambient noise...")
    with microphone as source:
        stt.adjust_for_ambient_noise(source)
    print("Calibration complete!")
    
    # Test TTS first
    print("\nTesting TTS system...")
    tts.say("Hello! Text to speech is working correctly.")
    tts.runAndWait()
    
    # Test STT + TTS pipeline
    print("\nTesting STT + TTS Pipeline...")
    print("Please say something when prompted...")
    
    for i in range(3):
        print(f"\nTest {i+1}/3: Say something...")
        
        try:
            with microphone as source:
                audio = stt.listen(source, timeout=10)
            
            print("Processing speech...")
            text = stt.recognize_google(audio)
            print(f"You said: {text}")
            
            # Generate simple response
            response = f"I heard you say: {text}. Thank you for testing!"
            print(f"Robot responds: {response}")
            
            # Speak response
            tts.say(response)
            tts.runAndWait()
            
            print("✓ Test passed!")
            
        except sr.WaitTimeoutError:
            print("✗ Test failed: No speech detected")
        except sr.UnknownValueError:
            print("✗ Test failed: Could not understand audio")
        except Exception as e:
            print(f"✗ Test failed: {e}")
    
    print("\n" + "=" * 50)
    print("INTEGRATION TEST COMPLETED")
    print("=" * 50)

def create_test_launch_file():
    """Create a ROS launch file for testing"""
    launch_content = """
<launch>
    <!-- Speech Dialogue Node -->
    <node name="speech_dialogue_node" pkg="your_package_name" type="speech_dialogue_node.py" output="screen">
        <param name="emotion_context" value="neutral" />
        <param name="continuous_listening" value="true" />
        <param name="debug_mode" value="true" />
    </node>
    
    <!-- Test emotion publisher (for testing) -->
    <node name="emotion_test_publisher" pkg="rostopic" type="rostopic" 
          args="pub -r 1 /emotion_detection std_msgs/String 
          '{\"dominant_emotion\": \"happy\", \"confidence\": 0.85}'" />
    
    <!-- Monitor topics -->
    <node name="topic_monitor" pkg="rostopic" type="rostopic" args="echo /robot_response" />
    
    <!-- RQT Graph for visualization -->
    <node name="rqt_graph" pkg="rqt_graph" type="rqt_graph" />
</launch>
"""
    
    with open("test_speech_dialogue.launch", "w") as f:
        f.write(launch_content.strip())
    
    print("Created test_speech_dialogue.launch file")

if __name__ == "__main__":
    try:
        # Check if running in ROS environment
        if 'ROS_MASTER_URI' in os.environ:
            # Run as ROS node
            node = SpeechDialogueROSNode()
            rospy.spin()
        else:
            # Run standalone integration test
            print("ROS environment not detected. Running standalone test...")
            test_stt_tts_integration()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Speech Dialogue Node shutting down")
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        if 'rospy' in globals():
            rospy.logerr(f"Node error: {e}")
```

### Quick Start Testing
```bash
# Install dependencies
pip install speechrecognition pyttsx3 pyaudio

# Run the pipeline
python stt_tts_pipeline.py

# Select testing option from menu
```
### ROS Integration Testing

```bash
# Launch ROS node
rosrun speech_dialogue speech_dialogue_node.py

# Test emotion input
rostopic pub /emotion_detection std_msgs/String "happy"

# Monitor responses
rostopic echo /robot_response
```
This implementation provides Dexter with a complete, testable STT + TTS system that integrates seamlessly with the robot's emotion detection and ROS architecture, fulfilling all requirements for the Speech & Dialogue Engineer role.
