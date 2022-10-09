
from playsound import playsound

from gtts import gTTS


def Text_to_speech(message): #call this to save the audio file 
    speech = gTTS(text = message)
    speech.save('pause1.mp3')
    playsound('pause1.mp3')


Text_to_speech("paused")