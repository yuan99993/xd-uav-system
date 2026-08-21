# One aircraft process incarnation. session_uuid must change whenever the
# onboard mission bridge restarts, so a decision service cannot accidentally
# dispatch a task to an old process that has the same uav_id.
string mission_id
string uav_id
string session_uuid
