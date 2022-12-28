# Equivalent to make maliput::api::Lane::id().
LaneId id
# Equivalent to make maliput::api::Lane::segment()->id().
SegmentId segment_id
# Equivalent to make maliput::api::Lane::index().
int32 index
# Equivalent to make maliput::api::Lane::to_left().
LaneId left_lane
# Equivalent to make maliput::api::Lane::to_right().
LaneId right_lane
# Equivalent to make maliput::api::Lane::length().
float64 length
# Equivalent to make maliput::api::Lane::GetBranchPoint(maliput::api::LaneEnd::Which::kStart)->id()
BranchPointId start_branch_point_id
# Equivalent to make maliput::api::Lane::GetBranchPoint(maliput::api::LaneEnd::Which::kFinish)->id()
BranchPointId finish_branch_point_id
# Equivalent to make maliput::api::Lane::GetDefaultBranch(maliput::api::LaneEnd::Which::kStart).
LaneEnd default_start_branch
# Equivalent to make maliput::api::Lane::GetDefaultBranch(maliput::api::LaneEnd::Which::kFinish).
LaneEnd default_finish_branch
