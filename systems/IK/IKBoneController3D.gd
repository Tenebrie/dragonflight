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

	var current_to = skeleton.global_transform.inverse() * get_child(0).global_position

	# TODO: No need to recalculate everything every frame
	for i in range(depth):
		var parent = skeleton.get_bone_parent(current_bone)
		if parent == -1:
			push_warning("[IKBoneController3D] Bone '%s' (depth = %d) has no parent. Depth too high?" % [skeleton.get_bone_name(current_bone), i])
			return
		
		var basis = skeleton.get_bone_global_pose(current_bone).basis

		var from = skeleton.get_bone_global_pose(current_bone).origin
		var to = from + basis.y if bones.is_empty() else current_to
		# If it's the first bone, length is not available
		var length = 0.1 if bones.is_empty() else to.distance_to(from)
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

	# TODO: Make this global down, not skeleton down
	gravity_vector = Vector3.DOWN * skeleton.global_transform.inverse()

	fabrik_phase1()
	fabrik_phase2()

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

func fabrik_phase1():
	var target_pos = skeleton.global_transform.inverse() * get_child(0).global_position

	for i in range(depth):
		var bone = bones[i]
		# Apply collision detection on the target position relative to the bone’s current “from” position.
		bone["to"] = move_and_collide(bone["from"], bone["to"], target_pos, bone["data_node"])
		# bone["to"] = target_pos

		var desired_from = bone["from"]
		# desired_from = apply_bias(bone["to"], desired_from, bone["length"], bone["parent_basis"].y, bone["stiffness"])
		# desired_from = apply_bias(bone["to"], desired_from, bone["length"], -gravity_vector, bone["gravity"])

		# Preserve bone length using the collision-adjusted target.
		bone["from"] = apply_bias(bone["to"], desired_from, bone["length"], Vector3.ZERO, 0.0)

		target_pos = bone["from"]

func fabrik_phase2():
	var source_pos = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).origin

	for i in range(depth):
		var bone = bones[depth - i - 1]
		bone["from"] = source_pos

		var desired_to = bone["to"]
		# desired_to = apply_bias(bone["from"], desired_to, bone["length"], bone["parent_basis"].y, bone["stiffness"])
		# desired_to = apply_bias(bone["from"], desired_to, bone["length"], -gravity_vector, bone["gravity"])

		# Integrate collision detection to adjust the desired target.
		# desired_to = move_and_collide(bone["from"], bone["to"], desired_to, bone["data_node"])
		
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

# No, it's not the real one ^^
func move_and_collide(current_from: Vector3, current_to: Vector3, target_to: Vector3, data_node: BoneAttachment3D) -> Vector3:
	if not data_node:
		return target_to

	var safe_point = get_safe_point(current_from, current_to, target_to, data_node)
	return safe_point

func get_safe_point(current_from: Vector3, current_to: Vector3, target_to: Vector3, data_node: BoneAttachment3D) -> Vector3:
	if not data_node:
		return target_to

	var body: RigidBody3D = data_node.get_child(0)
	var shape: CollisionShape3D = body.get_child(0)

	var new_transform: Transform3D = data_node.global_transform.looking_at(skeleton.global_transform * target_to, Vector3.UP)

	# new_transform.origin = new_transform * shape.position

	new_transform.basis = new_transform.basis.rotated(new_transform.basis.x, PI / 2)
	new_transform.origin += new_transform.basis * -shape.position

	# print(new_transform.origin)

	# var params = PhysicsTestMotionParameters3D.new()
	# params.from = body.global_transform
	# params.motion = skeleton.global_transform * target_pos - skeleton.global_transform * current_pos
	# params.motion = skeleton.global_transform
	# var result: PhysicsTestMotionResult3D = PhysicsTestMotionResult3D.new()
	# PhysicsServer3D.body_test_motion(body.get_rid(), params, result)
	# if result.get_collision_count() > 0:
		# print(result.get_collision_safe_fraction())
		# return current_pos + params.motion * result.get_collision_safe_fraction()
	
	var query = PhysicsShapeQueryParameters3D.new()
	query.shape = shape.shape
	query.transform = new_transform
	var result = get_world_3d().direct_space_state.intersect_shape(query)
	# print(result)
	if result.size() > 0:
		return current_to
		
		var collision = result[0]
		var collision_point = collision.position
		var collision_normal = collision.normal
		var safe_point = collision_point + collision_normal * 0.1

		# print("Collision detected at: ", collision_point)
		# print("Safe point: ", safe_point)
		return safe_point

	return target_to

func apply_bias(original_from: Vector3, original_to: Vector3, length: float, bias_to: Vector3, influence: float) -> Vector3:
	var desired_direction = (original_to - original_from).normalized()
	var desired_to = original_from + desired_direction * length

	var biased_to = (original_from + bias_to * length)
	var biased_position = (1 - influence) * desired_to + influence * biased_to
	var biased_direction = (biased_position - original_from).normalized()

	return original_from + biased_direction * length

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
