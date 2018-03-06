# Message for sending storybook info back to the controller,
# e.g. what word was tapped, what the current sentence is, etc.

Header header

int8 event_type

string message

# Event types that can be received from the tablet app.
int8 HELLO_WORLD = 0
int8 SPEECH_ACE_RESULT = 1
int8 REQUEST_ROBOT_FEEDBACK = 2
int8 WORD_TAPPED = 3
int8 SCENE_OBJECT_TAPPED = 4