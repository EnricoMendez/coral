import pyaudio
import wave
import speech_recognition as sr

# Configuración de parámetros de audio
FORMAT = pyaudio.paInt16  # Formato de audio
CHANNELS = 1  # Número de canales de audio (mono: 1, estéreo: 2)
RATE = 16000  # Tasa de muestreo (número de muestras por segundo)
CHUNK = 1024  # Tamaño del búfer de audio
RECORD_SECONDS = 2  # Duración de la grabación en segundos
WAVE_OUTPUT_FILENAME = "grabacion.wav"  # Nombre del archivo de salida WAV

# Inicializar PyAudio
audio = pyaudio.PyAudio()

# Inicializar el reconocedor de voz de Sphinx
recognizer = sr.Recognizer()

# Función para reconocer el audio grabado
def reconocer_audio(filename):
    with sr.AudioFile(filename) as source:
        audio_data = recognizer.record(source)
        try:
            text = recognizer.recognize_sphinx(audio_data)
            print("Texto reconocido: ", text)
        except sr.UnknownValueError:
            print("No se pudo reconocer el audio")
        except sr.RequestError as e:
            print("Error al procesar el audio: ", e)

# Iniciar la grabación de audio
stream = audio.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

print("Grabando audio...")

frames = []  # Lista para almacenar los fragmentos de audio

# Grabar audio durante el tiempo especificado
for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Grabación finalizada.")

# Detener la grabación
stream.stop_stream()
stream.close()
audio.terminate()

# Guardar los datos grabados en un archivo WAV
wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(audio.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()

print("Archivo WAV guardado: ", WAVE_OUTPUT_FILENAME)

# Reconocer el audio grabado utilizando Sphinx
print("Reconociendo audio...")
reconocer_audio(WAVE_OUTPUT_FILENAME)
