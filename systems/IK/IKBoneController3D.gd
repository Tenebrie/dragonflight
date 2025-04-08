@tool

extends Node3D

class_name IKBoneController3D
## Fabrik-based inverse kinematics for 3D bones

@export_range(1, 100) var depth: int = 1

var enabled: bool = true
@export var simulation_enabled: bool:
	get:
		return enabled
	set(value):
		skeleton = get_parent()
		enabled = value
		if skeleton:
			skeleton.reset_bone_poses()

var preview: bool = false
@export var editor_preview: bool:
	get:
		return preview
	set(value):
		skeleton = get_parent()
		preview = value
		if skeleton:
			skeleton.reset_bone_poses()

@export_subgroup("Root")
@export_range(0, 1) var root_stiffness: float = 0.2
@export_range(-1, 1) var root_pitch: float = 0.0
@export_range(-1, 1) var root_yaw: float = 0.0
@export_range(-1, 1) var root_roll: float = 0.0

@export_subgroup("Bones")
@export_range(0, 1) var gravity: float = 0.0
@export_range(0, 0.2) var bone_weight: float = 0.0

@export_range(0, 1) var stiffness: float = 0.0
@export_range(0, 0.2) var stiffness_per_bone: float = 0.0

@export_range(0, 1) var rotational_stiffness: float = 0.0

var skeleton: Skeleton3D
var bones: Array[Dictionary] = []

var gravity_vector: Vector3

# Debug
var debug_mesh: MeshInstance3D

@export_subgroup("Debug")
var bone_debug: bool = false
@export var editor_bone_debug: bool:
	get:
		return bone_debug
	set(value):
		bone_debug = value
		if not value and debug_mesh:
			debug_mesh.queue_free()
			debug_mesh = null
		elif value:
			debug_mesh = MeshInstance3D.new()
			debug_mesh.top_level = true
			var mesh = ImmediateMesh.new()
			debug_mesh.mesh = mesh
			var material = StandardMaterial3D.new()
			material.vertex_color_use_as_albedo = true
			material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
			debug_mesh.material_override = material
			add_child(debug_mesh)
# Code

func _ready() -> void:
	print(simulation_enabled)
	skeleton = get_parent()

func _process(_delta: float) -> void:
	for i in range(get_child_count()):
		var child = get_child(i)
		if child.get_class() != "BoneAttachment3D" || child.bone_idx == -1:
			continue
		var bone_transform: Transform3D = skeleton.get_bone_global_pose(child.bone_idx)
		child.global_position = bone_transform.origin * skeleton.global_transform.inverse()

func _physics_process(_delta: float) -> void:
	if get_child_count() == 0:
		return
	var first_child: BoneAttachment3D = get_child(1)
	var bone_idx = first_child.bone_idx
	if not enabled:
		return
	if not preview and Engine.is_editor_hint():
		return
	if not skeleton:
		return
	if bone_idx == -1:
		return
	if depth == 0:
		return

	bones = []
	var current_bone = bone_idx

	var current_to = Vector3.ZERO

	# TODO: No need to recalculate everything every frame
	for i in range(depth):
		var parent = skeleton.get_bone_parent(current_bone)
		if parent == -1:
			push_warning("[IKBoneController3D] Bone '%s' (depth = %d) has no parent. Depth too high?" % [skeleton.get_bone_name(current_bone), i])
			return
		
		var bone_transform = skeleton.get_bone_global_pose(current_bone)
		var from = bone_transform.origin
		var to = from + bone_transform.basis.y.normalized() if bones.is_empty() else current_to
		# If it's the first bone, length is not available
		var length = 1
		if i > 0:
			var rest_pose = skeleton.get_bone_global_rest(current_bone)
			var child_rest_pose = skeleton.get_bone_global_rest(bones[i - 1]["idx"])
			length = rest_pose.origin.distance_to(child_rest_pose.origin)

		var parent_basis = skeleton.get_bone_global_pose(parent).basis
		var rest_basis = skeleton.get_bone_global_rest(current_bone).basis

		if i == depth - 1:
			parent_basis = parent_basis.rotated(parent_basis.z, root_yaw * PI).rotated(parent_basis.x, -root_pitch * PI)

		var node
		var node_name = skeleton.get_bone_name(current_bone).replace(".", "_")
		if find_child(node_name):
			node = find_child(node_name)

		var bone = {
			"idx": current_bone,
			"parent_idx": parent,
			"from": from,
			"to": to,
			"length": length,
			"gravity": clamp(gravity + bone_weight * i, 0, 1),
			"stiffness": clamp(stiffness + stiffness_per_bone * i, 0, 1),
			# "gravity": -1,
			# "stiffness": 0.2 + 0.4 * i,
			"basis": basis,
			"parent_basis": parent_basis,
			"rest_basis": rest_basis,
			"data_node": node,
		}

		if i == depth - 1:
			bone["stiffness"] = root_stiffness

		bones.append(bone)
		current_bone = parent
		current_to = from

	gravity_vector = skeleton.global_transform.inverse() * Vector3.DOWN

	debug_clear_mesh()

	var target_pos = skeleton.global_transform.inverse() * get_child(0).global_position
	fabrik_phase1(target_pos, 0, depth)

	debug_print_bones(Color.GREEN)

	var source_pos = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).origin
	fabrik_phase2(source_pos, 0, depth)

	# Apply
	bones.reverse()

	for i in range(depth):
		var bone = bones[i]
		var parent_transform: Transform3D = skeleton.get_bone_global_pose(bone["parent_idx"])
		var new_transform: Transform3D = skeleton.get_bone_global_pose(bone["idx"])

		new_transform.origin = bone["from"]

		if i == 0:
			parent_transform = parent_transform.rotated(parent_transform.basis.y, root_roll * PI)

		var forward: Vector3 = (bone["to"] - bone["from"]).normalized()

		var old_up = new_transform.basis.z.rotated(forward, -PI / 2)
		var preferred_up = (old_up - forward * old_up.dot(forward)).normalized()

		var twist = get_roll_difference(new_transform, parent_transform)

		var untwisted_up = preferred_up.rotated(forward, twist)
		preferred_up = preferred_up.lerp(untwisted_up, 1)

		preferred_up = old_up.lerp(preferred_up, 1 - rotational_stiffness)
	
		var up = forward.cross(preferred_up).normalized()
		var normal = forward.cross(up).normalized()
		
		new_transform.basis = Basis(normal, forward, up).orthonormalized()
		skeleton.set_bone_global_pose(bone["idx"], new_transform)

# New bias idea. Instead of applying bias to the bone, do something funky with drawing a curve and following it or whatever.
func fabrik_phase1(target_pos: Vector3, from_index: int, to_index: int):
	for i in range(from_index, to_index):
		var bone = bones[i]

		bone["to"] = move_and_collide(bone["from"], bone["to"], target_pos, bone["data_node"])

		# If we failed to move the bone, we have to fix the chain
		if bone["to"].distance_to(target_pos) > 0.01:
			fabrik_phase2(bone["to"], 0, i)

		var desired_from = bone["from"]
		# desired_from = apply_bias(bone["to"], desired_from, bone["length"], bone["parent_basis"].y, bone["stiffness"])
		# desired_from = apply_bias(bone["to"], desired_from, bone["length"], gravity_vector, bone["gravity"])

		var safe_from = move_and_collide(bone["to"], bone["from"], desired_from, bone["data_node"])

		# Preserve bone length using the collision-adjusted target.
		bone["from"] = apply_bias(bone["to"], safe_from, bone["length"], Vector3.ZERO, 0.0)

		target_pos = bone["from"]

func fabrik_phase2(source_pos: Vector3, from_index: int, to_index: int):
	for i in range(to_index - 1, from_index - 1, -1):
		var bone = bones[i]
		bone["from"] = source_pos

		var desired_to = bone["to"]
		# desired_to = apply_bias(bone["from"], desired_to, bone["length"], bone["parent_basis"].y, bone["stiffness"])
		# desired_to = apply_bias(bone["from"], desired_to, bone["length"], gravity_vector, bone["gravity"])

		var actual_bone = skeleton.get_bone_global_pose(bone["idx"])
		var actual_to = actual_bone.origin + actual_bone.basis.y * bone["length"]
		# desired_to = move_and_collide(bone["from"], actual_to, desired_to, bone["length"], bone["data_node"])

		desired_to = move_and_collide(bone["from"], actual_to, desired_to, bone["data_node"])

		# Preserve bone length using the collision-adjusted desired target.
		bone["to"] = apply_bias(bone["from"], desired_to, bone["length"], Vector3.ZERO, 0.0)

		source_pos = bone["to"]


# func apply_rot_constraints():
# 		# Rotation limit in every axis (yaw, pitch, roll)
# 	var limit = PI / 4

# 	# Get the rest direction (forward vector in rest pose)
# 	var rest_direction = rest_basis.y

# 	# Calculate the angle between desired and rest direction
# 	var angle = rest_direction.angle_to(desired_direction)

# 	# If we're within the limit, return the desired direction
# 	if angle <= limit:
# 		return {
# 			"direction": desired_direction,
# 			"pitch": 0,
# 			"roll": 0,
# 			"yaw": 0,
# 		}

# 	# Calculate the rotation axis (cross product of rest and desired direction)
# 	var rotation_axis = rest_direction.cross(desired_direction).normalized()

# 	# Create a basis that rotates from rest_direction to desired_direction
# 	var rotation_basis = Basis(rotation_axis, limit)
	
# 	# Apply the limited rotation to the rest direction
# 	var constrained_direction = rotation_basis * rest_direction

# 	# Calculate the actual angles for debugging/visualization
# 	var constrained_basis = Basis(rest_direction, constrained_direction)
# 	var euler = constrained_basis.get_euler()
	
# 	return {
# 		"direction": constrained_direction,
# 		"pitch": euler.x,
# 		"roll": euler.z,
# 		"yaw": euler.y,
	# }

func apply_bias(original_from: Vector3, original_to: Vector3, length: float, bias_to: Vector3, influence: float, depth: int = 0) -> Vector3:
	if depth > 5:
		push_warning("[apply_bias] Depth limit reached")
		return original_to

	var desired_direction = (original_to - original_from).normalized()
	var desired_to = original_from + desired_direction * length

	var biased_to = (original_from + bias_to * length)
	var biased_position = (1 - influence) * desired_to + influence * biased_to
	var biased_direction = (biased_position - original_from).normalized()

	var target = original_from + biased_direction * length

	# var collision_result = raycast_collide(original_from, target)
	# if collision_result["collision"]:
	# 	var from = collision_result["point"] + (collision_result["point"] - original_from).normalized() * length
	# 	return apply_bias(original_from, collision_result["point"], length, Vector3.ZERO, 0.0, depth + 1)

	return target

func raycast_collide(from: Vector3, to: Vector3) -> Dictionary:
	var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.new()
	query.from = skeleton.global_transform * from
	query.to = skeleton.global_transform * to

	var result = get_world_3d().direct_space_state.intersect_ray(query)
	if result.size() > 0:
		return {
			"point": skeleton.global_transform.inverse() * result.position,
			"collision": true,
		}

	return {
		"point": to,
		"collision": false,
	}

func move_and_collide(sample_origin: Vector3, from: Vector3, to: Vector3, data_node: BoneAttachment3D) -> Vector3:
	var data = get_safe_point_for_line(sample_origin, from, to, data_node)

	return data["point"]

func get_safe_point_for_line(skeleton_sample_origin: Vector3, skeleton_from: Vector3, skeleton_to: Vector3, data_node: BoneAttachment3D) -> Dictionary:
	var sample_count = 10

	var sample_origin = skeleton.global_transform * skeleton_sample_origin
	var sample_start = skeleton.global_transform * skeleton_from
	var sample_end = skeleton.global_transform * skeleton_to

	var safe_point_found = false
	var last_known_safe_point = null
	for i in range(sample_count):
		var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.new()
		query.from = sample_origin
		query.to = sample_start.lerp(sample_end, float(i + 1) / sample_count)

		var result = get_world_3d().direct_space_state.intersect_ray(query)
		if result.size() > 0:
			if safe_point_found:
				return {
					"point": skeleton.global_transform.inverse() * last_known_safe_point,
					"resolved": true,
				}
			else:
				continue

		safe_point_found = true
		last_known_safe_point = query.to

	if safe_point_found:
		return {
			"point": skeleton.global_transform.inverse() * last_known_safe_point,
			"resolved": true,
		}

	# push_warning("No safe point found")
	return {
		"point": skeleton_from,
		"resolved": false,
	}

# Returns the roll difference (in radians) between transform1 and transform2,
# measured around the provided twist_axis.
func get_roll_difference(transform1: Transform3D, transform2: Transform3D) -> float:
	# Assume transform1 and transform2 are your transforms
	# and that their basis have the axes:
	# forward: Y, up: X
	# Choose the forward axis from transform1
	var forward = transform1.basis.y.normalized()

	# Get the up vectors from each transform
	var up1 = transform1.basis.x
	var up2 = transform2.basis.x

	# Project up vectors onto plane perpendicular to 'forward'
	up1 = (up1 - forward * up1.dot(forward)).normalized()
	up2 = (up2 - forward * up2.dot(forward)).normalized()

	# Compute the angle between the projected up vectors
	var dot_val = clamp(up1.dot(up2), -1, 1)
	var angle = acos(dot_val)

	# Determine the sign using the cross product
	if forward.dot(up1.cross(up2)) < 0:
		angle = - angle

	return angle

func debug_clear_mesh():
	if not debug_mesh:
		return
	debug_mesh.mesh.clear_surfaces()

func debug_print_single_bone(from: Vector3, to: Vector3, color: Color):
	if not debug_mesh:
		return
	debug_mesh.mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	debug_mesh.mesh.surface_set_color(color)
	debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * from)
	debug_mesh.mesh.surface_set_color(color)
	debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * to)
	debug_mesh.mesh.surface_end()

func debug_print_bones(color: Color):
	if not debug_mesh:
		return
	for bone in bones:
		debug_mesh.mesh.surface_begin(Mesh.PRIMITIVE_LINES)
		debug_mesh.mesh.surface_set_color(color)
		debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * bone["from"])
		debug_mesh.mesh.surface_set_color(color)
		debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * bone["to"])
		debug_mesh.mesh.surface_end()
