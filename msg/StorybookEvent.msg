# Message for sending storybook info back to the controller,
# e.g. what word was tapped, what the current sentence is, etc.

Header header

int8 event_type

string message

# Event types that can be received from the tablet app.
int8 HELLO_WORLD = 0 # There is no message.
int8 SPEECH_ACE_RESULT = 1 # Message is json string of result
int8 WORD_TAPPED = 2 # Message is {index: int, word: string}
int8 SCENE_OBJECT_TAPPED = 3 # Message is {id: int, label: string}
int8 SENTENCE_SWIPED = 4 # Message is {index: int, text: string}
int8 RECORD_AUDIO_COMPLETE = 5 # Message is the {index: int} of the sentence
