# Raw audio chunk for streaming
# Published by: Voice Manager
# Subscribed by: Cloud Bridge

uint8[] data              # Raw audio data
string format             # e.g., "pcm_s16le"
int32 sample_rate         # e.g., 16000
int32 channels            # e.g., 1
builtin_interfaces/Time timestamp
string session_id
