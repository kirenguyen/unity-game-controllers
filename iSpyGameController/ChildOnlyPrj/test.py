import json



jibo_tts_data = "../res/jibo_speech.json"
jibo_tts_file = open(jibo_tts_data)
jibo_tts_dict = json.loads(jibo_tts_file.read())



#print(jibo_tts_dict)
print(jibo_tts_dict['hint'])