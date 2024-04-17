#Motor control signal (not used, left here for now just in case)
#float64 linear_x
#float64 angular_z

#Servo control signal
int64 tilt_angle
int64 pan_angle

#controller signal, only the fire button (square)
# square = int64 button[2], fire if 1, nothing if 0
int64 fire
