# Message for sending commands to the storybook app.
# Note that if structure of this message is changed, will need to update
# RosbridgeUtilities deserialization process in Unity app.

Header header

int8 command

# Any optional parameters, sent as a string of serialized JSON.
string params

int8 PING_TEST = 0 # Dummy message that simply pings the storybook.
int8 HIGHLIGHT_WORD = 1 # Params is which index word to highlight
int8 HIGHLIGHT_SCENE_OBJECT = 2 # Params is which id scene object to highlight
int8 HIGHLIGHT_STANZA = 3 # Params is which index stanza to highlight

