# Internal detector transaction completion. APPLIED follows batch-boundary commit.
std_msgs/Header header
string request_id
string session_id
bool success
string error_code
string message
string model_id
string source_sha256
string backend
uint64 model_epoch
string[] class_names
