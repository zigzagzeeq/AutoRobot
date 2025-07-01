def respond_to_emotion(emotion_dict):
    """
    Generate a response based on the detected emotions.
    If there is a tie, respond with mixed emotions.
    If neutral is the highest, ignore it and focus on other emotions.
    """
    # dictionary preprocessing
    # Remove neutral if it exists
    if 'neutral' in emotion_dict:
        del emotion_dict['neutral']
    
    # If no emotions left after removing neutral
    if not emotion_dict:
        return "No emotion detected.", False
    
    # Find the list of emotions with highest frequency
    # Sort emotions by frequency
    sorted_emotions = sorted(emotion_dict.items(), key=lambda x: x[1], reverse=True)
    highest_freq = sorted_emotions[0][1]

    # Check for ties
    tied_emotions = [emotion for emotion in sorted_emotions if emotion[1] == highest_freq]
    
    if len(tied_emotions) > 1:
        response = "I feel like you all are feeling mixed emotions like " + ", ".join([emotion[0] for emotion in tied_emotions])

    else:
        response = "I can detect most of you here are feeling " + tied_emotions[0][0] +". "

    response += "What made you feel that way? Please tell me in a few words."
    return response, True

def generate_emotion_response(self, emotion_dict):
        """Generate appropriate response based on detected emotion"""
        if 'neutral' in emotion_dict:
            del emotion_dict['neutral']
        
        # If no emotions left after removing neutral
        if not emotion_dict:
            return "No emotion detected.", False
        
        # Find the list of emotions with highest frequency
        # Sort emotions by frequency
        sorted_emotions = sorted(emotion_dict.items(), key=lambda x: x[1], reverse=True)
        highest_freq = sorted_emotions[0][1]

        # Check for ties
        tied_emotions = [emotion for emotion in sorted_emotions if emotion[1] == highest_freq]
    
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
            emotion = random.choice(tied_emotions)[0]
            # Return a mixed emotion response
            return f"I feel like you all are feeling mixed emotions like {emotion_list}. Let's focus on those are feeling with {emotion}. What made you feel that way? Please tell me in a few words.", True
        elif len(tied_emotions) == 1:
            emotion = tied_emotions[0][0]
            # Return a random response based on the detected emotion
        
        return random.choice(responses.get(emotion, ["I can sense your emotions. Let's talk about how you're feeling."])), True

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

