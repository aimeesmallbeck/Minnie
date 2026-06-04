# Map metadata for the map library
string id           # Unique identifier (directory name)
string name         # Human-readable name
string description  # Optional description
string type         # "aimee_nav" or "ros_map"
float64 resolution  # Meters per cell
float64 width_m     # Map width in meters
float64 height_m    # Map height in meters
float64 origin_x    # Map origin X
float64 origin_y    # Map origin Y
bool has_waypoints  # Whether waypoints are defined
string created_at   # ISO 8601 timestamp
string updated_at   # ISO 8601 timestamp
