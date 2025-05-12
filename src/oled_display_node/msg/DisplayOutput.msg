# Display output action

# Action type definitions for user consumption
uint8 DISPLAY_ALL = 1
uint8 DISPLAY_SUBSTRING = 2
uint8 DISPLAY_STARTUP_STRING = 10
uint8 DISPLAY_SET_BRIGHTNESS = 20

uint8 action_type
int8 row
int8 column
int8 num_chars
uint8 attributes
string text
string comment
