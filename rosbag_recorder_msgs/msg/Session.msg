# Session description, recorded once at start (event START) and once at end (event END).
uint8 START=0
uint8 END=1

uint8 event
string session_id
string label
string notes
string profile
string[] profile_chain
# Resolved profile: topics, sources and options.
string profile_yaml
string config_dir
string bag_path
string host
string user
string ros_distro
string rmw_implementation
string ros_domain_id
# Node parameters the session ran with.
string parameters_yaml
# git state of the configured workspace and its submodules.
string git_yaml
FileSnapshot[] params
string[] graph_nodes
# "name type" per topic on the graph at start.
string[] graph_topics
CheckResult[] preflight
builtin_interfaces/Time start_time
# Set in the END message only.
builtin_interfaces/Time end_time
string end_reason
uint64 bag_bytes
uint32 markers
