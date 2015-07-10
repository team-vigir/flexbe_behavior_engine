# Behavior Status
uint8 STARTED = 0
uint8 FINISHED = 1
uint8 FAILED = 2

uint8 LOCKED = 4
uint8 WAITING = 5
uint8 SWITCHING = 6

# General problems
uint8 WARNING = 10
uint8 ERROR = 11

# Engine Status
uint8 READY = 20

time stamp
int32 behavior_id
uint8 code

string[] args
