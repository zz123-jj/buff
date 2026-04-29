# This message contains auto aiming data

std_msgs/Header header

float32 pitch
float32 yaw

# auto aiming target, see enum below
uint8 target_id

uint8 TARGET_ID0_NONE                       = 0
uint8 TARGET_ID1_HERO                       = 1
uint8 TARGET_ID2_ENGINEER                   = 2
uint8 TARGET_ID3_INFANTRY                   = 3
uint8 TARGET_ID4_INFANTRY                   = 4
uint8 TARGET_ID5_INFANTRY                   = 5
uint8 TARGET_ID6_OUTPOST                    = 6
uint8 TARGET_ID7_SENTRY                     = 7

# auto aiming target distance
float32 target_distance

# auto aiming mode, see enum below
uint8 vision_mode

uint8 VISION_MODE_ARMOR                     = 1
uint8 VISION_MODE_SMALL                     = 2
uint8 VISION_MODE_BIG                       = 3
uint8 VISION_MODE_TAG                       = 4

# auto aiming shoot command, see enum below
uint8 shoot_command

uint8 CEASE_FIRE                            = 0
uint8 ALLOW_SHOOT                           = 1

# shoot mode, if auto, then the robot will shoot continuously when the target is detected, if single, then the robot will shoot once when the target is detected, and then wait for the next command
uint8 shoot_mode

uint8 SHOOT_MODE_AUTO = 1
uint8 SHOOT_MODE_SINGLE = 0
