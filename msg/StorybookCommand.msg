# Message for sending commands to the storybook app.
# Note that if structure of this message is changed, will need to update
# RosbridgeUtilities deserialization process in Unity app.

Header header

int8 command

# Any optional parameters, sent as a string of serialized JSON.
string params

int8 HELLO_WORLD_ACK = 0 # Initializtion info. Params is {participant_id: string, story_name: string}
int8 HIGHLIGHT_WORD = 1 # Params is {indexes:[int], unhighlight: bool, stay_on: bool}, words to highlight, whether it should stay on forever, or turn off
int8 HIGHLIGHT_SCENE_OBJECT = 2 # Params is {ids: [int]}, scene objects to highlight
int8 SHOW_NEXT_SENTENCE = 3 # Params is {index: int, child_turn: bool, record: bool}, sentence to highlight and whether it's the child's turn
int8 START_RECORD = 4 # Params is empty. Starts recording without attempting to reshow the sentence
int8 CANCEL_RECORD = 5 # Params is empty
int8 GO_TO_PAGE = 6 # Params is {page_number: int}
int8 NEXT_PAGE = 7 # Params is empty
int8 GO_TO_END_PAGE = 8 # Params is empty
int8 SHOW_LIBRARY_PANEL = 9 # Params is empty
int8 HIGHLIGHT_ALL_SENTENCES = 10 # Params is empty