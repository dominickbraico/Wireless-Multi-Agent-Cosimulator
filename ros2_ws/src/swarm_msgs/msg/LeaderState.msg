# Leader state broadcast over the emulated radio (the ONLY inter-agent message).
# Positions are in the shared world frame (ENU meters), NOT a PX4 local NED frame.
uint32 sender_id
uint64 seq                          # per-sender counter; gaps = loss
builtin_interfaces/Time stamp_sent  # sender wall clock at publish
float64[3] position                 # x (East), y (North), z (Up), meters
float64[3] velocity                 # m/s, same frame
