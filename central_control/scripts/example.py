from gtts import gTTS
import os

text = "Hello, its me"

tts = gTTS(text, lang='en-us')
tts.save('output.mp3')

os.system('mpg123 output.mp3 > /dev/null 2>&1')  # Reproduce el archivo de audio utilizando mpg123 o utiliza tu reproductor de audio preferido

