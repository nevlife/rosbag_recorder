# Quality of one recorded topic or source channel over the last report period.
uint8 OK=0
uint8 LOW=1
uint8 STALE=2
uint8 WAITING=3
uint8 NO_PUBLISHER=4
uint8 LATCHED_MISSING=5
uint8 TYPE_UNAVAILABLE=6
uint8 ABNORMAL=7

string name
string type
# Empty for a ROS topic, "flux" for a Flux input.
string source
uint8 status
bool required
bool latched
# 0 when the catalog gives no rate.
float64 expected_hz
float64 hz
# Seconds since the last message; NaN before the first one.
float64 age_s
float64 size_kb
uint64 received
# Handed to the writer. Its cache can still drop them: summary.yaml has the stored counts.
uint64 written
uint32 publishers
string[] issues
string[] values
