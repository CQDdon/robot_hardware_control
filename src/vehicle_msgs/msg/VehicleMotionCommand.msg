# Steering target in radians. The simple controller applies it to all steering joints.
float32 steering_angle_rad
# Requested wheel speed and braking effort, both inclusive [0, 100].
float32 speed_percentage
float32 brake_percentage
uint8 GEAR_NEUTRAL=0
uint8 GEAR_DRIVE=1
uint8 GEAR_REVERSE=2
uint8 gear
