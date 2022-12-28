# Equivalent to make maliput:api::RoadGeometry::id().
RoadGeometryId id
# Equivalent to iterate over all Junctions and retrieve their IDs.
JunctionId[] junction_ids
# Equivalent to iterate over all BranchPoints and retrieve their IDs.
BranchPointId[] branch_point_ids
# Equivalent to make maliput::api::RoadGeometry::linear_tolerance().
float64 linear_tolerance
# Equivalent to make maliput::api::RoadGeometry::angular_tolerance().
float64 angular_tolerance
# Equivalent to make maliput::api::RoadGeometry::scale_length().
float64 scale_length
# Equivalent to make maliput::api::RoadGeometry::inertial_to_backend_frame_translation().
geometry_msgs/Vector3 inertial_to_backend_frame_translation
