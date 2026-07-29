uint8 INVALID=0
uint8 MEASURED=1
uint8 PREDICTED=2
uint8 STALE=3

std_msgs/Header header
bool valid
uint8 filter_mode
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 confidence
uint32 measurement_age_ms
bool vision_detected
float32 center_u
float32 center_v
