# This message represents state that the storybook will publish
# consistently, at 10Hz.

Header header

bool audio_playing

# The app is in a storybook and progressing through pages (as opposed to still in story selection).
bool is_reading

# Either EXPLORE_MODE or EVALUATE_MODE
int8 storybook_mode

string current_story
int8 num_pages

# If evaluating or in playback, this will reflect the current stanza.
int8 current_stanza_index

# Optional, will be filled in during audio playback.
# E.g. if the 5th word (0-indexed) is currently playing, this will be 5.
int8 current_tinkertext_index

int8 EXPLORE_MODE = 0
int8 EVAlUATE_MODE = 1

#
# Other properties that can be filled in later...
#

# int8 reader_id 
