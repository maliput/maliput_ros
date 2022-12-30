# Equivalent to maliput::api::LaneEnd::Which::kStart.
int32 WHICHSTART=0
# Equivalent to maliput::api::LaneEnd::Which::kFinish.
int32 WHICHFINISH=1

LaneId lane_id
# It must be one of the above WHICH* constants to represent maliput::api::LaneEnd::Which.
int32 end
