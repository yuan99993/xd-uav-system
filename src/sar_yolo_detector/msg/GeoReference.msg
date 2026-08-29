# Structured coordinate reference shared by every aircraft in one mission.
uint8 UNKNOWN=0
uint8 LOCAL_ENU=1
uint8 ECEF=2
uint8 WGS84=3

uint8 coordinate_type
string epsg
string frame_id
string map_uuid

# Required for LOCAL_ENU. Altitude is ellipsoid height unless vertical_datum
# explicitly names a geoid/orthometric datum.
float64 origin_latitude_deg
float64 origin_longitude_deg
float64 origin_altitude_m
string vertical_datum
string geoid_model

string transform_version
time valid_from
time valid_until
