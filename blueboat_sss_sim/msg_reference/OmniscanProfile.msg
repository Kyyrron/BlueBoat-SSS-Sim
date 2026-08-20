# Reference copy of blueboat_interfaces/msg/OmniscanProfile.msg.
# The blueboat_sss_sim package REUSES the existing definition from
# blueboat_interfaces and does NOT define any new message. This copy is
# provided only so the simulator repo is self-describing; field set and
# order were confirmed against sss_node.py and a captured raw frame.
std_msgs/Header header
string side                    # "port" | "starboard"
uint32 ping_number
uint32 start_mm
uint32 length_mm
uint32 timestamp_ms            # device uptime clock [ms]
uint32 ping_hz                 # acoustic frequency (~451127 Hz)
uint16 gain_index
uint16 num_results
uint16 sos_dmps                # speed of sound [dm/s] (15000 = 1500 m/s)
uint8 channel_number
float32 pulse_duration_sec
float32 analog_gain
float32 max_pwr_db
float32 min_pwr_db
float32 transducer_heading_deg
float32 vehicle_heading_deg
uint16[] pwr_results           # linear echo power, num_results entries
