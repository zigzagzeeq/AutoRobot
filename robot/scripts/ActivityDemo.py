import time

class ActivityDemo:
    """Class to handle activity demonstrations based on user emotion"""
    
    def __init__(self, emotion = 'neutral'):
        self.emotion = emotion
        self.ifWanted = False
        self.activity = None
    
    def suggest_activities(self):
        """Suggest activities based on the detected emotion"""
        activities = {
            'sad': ['guided meditation', 'breathing exercise', 'gentle stretching', 'journaling prompts'],
            'angry': ['breathing exercise', 'progressive muscle relaxation', 'counting meditation', 'mindful walking'],
            'happy': ['gratitude exercise', 'energy boost activities', 'mindfulness practice', 'celebration ritual'],
            'fear': ['grounding exercise', 'breathing technique', 'calming meditation', 'anxiety relief'],
            'surprise': ['reflection exercise', 'mindfulness practice', 'processing technique', 'grounding exercise']
        }
        
        self.activity = activities.get(self.emotion, ['breathing exercise', 'mindfulness practice'])
        return self.activity
    
    def get_user_decision(self, ifWanted):
        """Get user's decision on whether to proceed with the suggested activity"""
        if ifWanted:
            self.speak("Great! Let's proceed with the activity.")
        else:
            self.speak("No problem. If you change your mind, just let me know.")
        self.ifWanted = ifWanted
        return ifWanted

    def initiate_activity(self):
        """Initiate the activity demonstration based on emotion"""
        if not self.ifWanted:
            return
        
        if self.activity == 'guided meditation':
            self.guided_meditation()
        elif self.activity == 'breathing exercise':
            self.breathing_exercise()
        elif self.activity == 'gentle stretching':
            self.gentle_stretching()
        elif self.activity == 'grounding exercise':
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
    
    def speak(self, message): 
        """Simulate speaking a message"""
        print(message)
        return message  # In a real application, this would use a text-to-speech engine