import speech_recognition as sr

# Crear un objeto de reconocimiento de voz
r = sr.Recognizer()


# Definir las palabras clave a reconocer
keywords = ["one", "two", "Three", "four", "five"]

# Obtener audio desde el micrófono
with sr.Microphone() as source:
    print("Init")
    r.adjust_for_ambient_noise(source)  # Ajustar para el ruido ambiental
    print('now you can talk')
    audio = r.listen(source)  # Escuchar el audio del micrófono

# Realizar el reconocimiento de voz utilizando Sphinx y las palabras clave definidas
try:
    text = r.recognize_sphinx(audio, keyword_entries=[(keywords[i], .95) for i in range(len(keywords))])
    print("Texto reconocido: " + text)
except sr.UnknownValueError:
    print("No se pudo reconocer ninguna de las palabras clave")
except sr.RequestError as e:
    print("Error al solicitar el servicio de reconocimiento de voz; {0}".format(e))
