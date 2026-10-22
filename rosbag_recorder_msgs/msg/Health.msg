# Recorder state and per-topic quality, published and recorded once per report period.
uint8 IDLE=0
uint8 PREVIEW=1
uint8 WAITING=2
uint8 RECORDING=3
uint8 PAUSED=4

builtin_interfaces/Time stamp
string session_id
string profile
uint8 state
float64 elapsed_s
string output_dir
# NaN when unknown.
float64 disk_free_gb
float64 disk_warn_gb
float64 disk_stop_gb
uint64 bag_bytes
float64 write_mbps
# Minutes until disk_stop_gb at the current write rate; NaN when unknown.
float64 remaining_min
string[] warnings
# Milliseconds the recorder spent on the previous 1 Hz tick (graph, health, preflight).
float64 tick_ms
TopicHealth[] topics
