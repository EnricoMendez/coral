import pyttsx3

engine = pyttsx3.init(driverName='espeak')
engine.setProperty('voice', 'google')  # Configura el motor de voz de Google Text-to-Speech

# Continuar con el uso de pyttsx3
engine.say("Hello! I am a voice from Google Text-to-Speech.")
engine.runAndWait()
