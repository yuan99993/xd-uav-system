std_msgs/Header header
string session_id
uint64 state_version
string mode                     # detect / track / fusion
string operating_state          # ACTIVE / WAITING_SELECTION / TRACKING / LOST
bool selection_active
string selected_target_uid
int32 selected_track_id
string selected_image_source
string requested_tracker_algorithm
string active_tracker_algorithm
string active_model_id
string active_model_sha256
string active_model_backend
uint64 model_epoch
bool model_ready
string last_error
string active_request_id
uint32 queued_commands
string transaction_state        # IDLE / EXECUTING / MODEL_PREPARING
