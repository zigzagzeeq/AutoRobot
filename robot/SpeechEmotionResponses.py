def respond_to_emotion(emotion_dict):
    """
    Generate a response based on the detected emotions.
    If there is a tie, respond with mixed emotions.
    If neutral is the highest, ignore it and focus on other emotions.
    """
    if not emotion_dict:
        return "No emotion detected."

    # Sort emotions by frequency
    sorted_emotions = sorted(emotion_dict.items(), key=lambda x: x[1], reverse=True)
    highest_freq = sorted_emotions[0][1]
    
    # Filter out neutral if it's the highest
    if sorted_emotions[0][0] == 'neutral' and len(sorted_emotions) > 1:
        sorted_emotions = [emotion for emotion in sorted_emotions if emotion[0] != 'neutral']
        if not sorted_emotions:
            return "No significant emotion detected."

    # Check for ties
    tied_emotions = [emotion for emotion in sorted_emotions if emotion[1] == highest_freq]
    
    if len(tied_emotions) > 1:
        response = "Mixed emotions detected: " + ", ".join([emotion[0] for emotion in tied_emotions])
    else:
        response = f"Detected emotion: {tied_emotions[0][0]} with confidence {tied_emotions[0][1]}"

    return response