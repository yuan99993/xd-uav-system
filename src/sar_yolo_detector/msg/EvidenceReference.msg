# A compact reference to operator-verifiable evidence. Images stay on the
# vehicle/recorder and are fetched only when requested over a suitable channel.
string evidence_uuid
string source_topic
time capture_stamp
string image_sha256
string retrieval_uri
sensor_msgs/RegionOfInterest roi
uint32 contour_vertex_count
