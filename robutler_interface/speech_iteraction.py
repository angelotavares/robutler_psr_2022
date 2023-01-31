import speech_recognition as sr
from gtts import gTTS
import os
import openai

# Initialize GPT-3 API key
openai.api_key = "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"

# Initialize recognizer class (for recognizing the speech)
r = sr.Recognizer()

# Reading Microphone as source
# listening the speech and store in audio_text variable
with sr.Microphone() as source:
    print(source.list_microphone_names())
    print("Talk")
    audio_text = r.listen(source)
    print("Time over, thanks")

# recoginize_() method will throw a request error if the API is unreachable
try:
    # using google speech recognition
    question = r.recognize_google(audio_text, language="en-US")
    print("Question: " + question)
except:
    print("Sorry, I did not get that")

# Use GPT-3 to generate a response to the question
response = openai.Completion.create(
    engine="text-davinci-003", prompt=(question + " "), temperature=0.5, max_tokens=2048
)

# Get the response text
text = response.choices[0].text

print(text)

# Create a gTTS object and save the response to an MP3 file
tts = gTTS(text=text, lang="en")
tts.save("response.mp3")

# Play the response
os.system("mpg321 response.mp3")
