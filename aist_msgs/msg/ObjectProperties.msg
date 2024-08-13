string                      object_name

# Primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[]        primitive_poses

# Visual meshes
string[]                    visual_mesh_urls
geometry_msgs/Pose[]        visual_mesh_poses
geometry_msgs/Vector3[]     visual_mesh_scales
std_msgs/ColorRGBA[]        visual_mesh_colors

# Collision meshes
shape_msgs/Mesh[]           collision_meshes
geometry_msgs/Pose[]        collision_mesh_poses
geometry_msgs/Vector3[]     collision_mesh_scales

# Subframes
string[]                    subframe_names
geometry_msgs/Pose[]        subframe_poses
