import speech_recognition as sr

# Crear un objeto de reconocimiento de voz
r = sr.Recognizer()

# Obtener audio desde el micrófono
with sr.Microphone() as source:
    print("Di algo...")
    audio = r.listen(source)  # Escuchar el audio del micrófono
    audio = r.listen(source, timeout=2)

    print('finish')

# Realizar el reconocimiento de voz utilizando el servicio de Google Speech Recognition
# try:
#     text = r.recognize_google(audio)
#     print("Texto reconocido: " + text)
# except sr.UnknownValueError:
#     print("No se pudo reconocer el audio")
# except sr.RequestError as e:
#     print("Error al solicitar el servicio de reconocimiento de voz; {0}".format(e))

# Realizar el reconocimiento de voz utilizando Sphinx
try:
    text = r.recognize_sphinx(audio)
    print("Texto reconocido: " + text)
except sr.UnknownValueError:
    print("No se pudo reconocer el audio")
except sr.RequestError as e:
    print("Error al solicitar el servicio de reconocimiento de voz; {0}".format(e))


