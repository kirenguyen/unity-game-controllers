# Message for sending commands to the storybook app.

Header header

int8 command

# Any optional parameters, sent as a string of serialized JSON.
string params

int8 PING_TEST = 0 # Dummy message that simply pings the storybook.