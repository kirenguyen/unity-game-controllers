# Message for sending storybook info back to the controller,
# e.g. what word was tapped, what the current sentence is, etc.

Header header

int8 message_type

string message

# Message types that can be received from the tablet app.
int8 HELLO_WORLD= 0
int8 SPEECH_ACE_RESULT = 1