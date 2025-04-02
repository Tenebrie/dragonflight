@tool

extends BoneAttachment3D

@export var target: Node3D
@export var skeleton: Skeleton3D
@export var pole_target: Node3D

var initial_rotation: Vector3

var preview: bool = false
@export var enable_preview: bool:
	get:
		return preview
	set(value):
		preview = value


func _ready() -> void:
	initial_rotation = rotation_degrees

var bones = []

var root_up: Vector3
var root_forward: Vector3
var gravity_vector: Vector3

func _physics_process(_delta: float) -> void:
	if not preview:
		return
	if not target or not target.is_inside_tree():
		return

	var depth = 3
	bones = []
	var current_bone = bone_idx

	# var orr = skeleton.get_bone_global_pose(bone_idx).origin
	# var target_position = lerp(orr, target.position, 0.5 * _delta)
	var target_position = target.position
	var current_to = target_position

	for i in range(depth):
		var parent = skeleton.get_bone_parent(current_bone)
		
		var self_basis = skeleton.get_bone_global_pose(current_bone).basis
		var from = skeleton.get_bone_global_pose(current_bone).origin
		var to = current_to
		# If it's the first bone, length is not relevant
		var length = 1.0 if bones.is_empty() else skeleton.get_bone_global_rest(current_bone).origin.distance_to(skeleton.get_bone_global_rest(parent).origin)
		var parent_basis = skeleton.get_bone_global_pose(parent).basis
		var rest_basis = skeleton.get_bone_global_rest(current_bone).basis
		bones.append({
			"idx": current_bone,
			"parent_idx": parent,
			"from": from,
			"to": to,
			"length": length,
			"pitch": 0.0,
			"yaw": 0.0,
			"roll": 0.0,
			"gravity": 0.0,
			"stiffness": 0.0,
			# "gravity": 0.05 + 0.05 * i,
			# "stiffness": 0.3 + 0.05 * i,
			# "gravity": -1,
			# "stiffness": 0.2 + 0.4 * i,
			"self_basis": self_basis,
			"parent_basis": parent_basis,
			"rest_basis": rest_basis
		})
		current_bone = parent
		current_to = from

	root_up = Vector3(0, 0, -1)
	root_forward = Vector3(0, -1, 0)
	# root_up = -skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).basis.z
	# root_forward = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).basis.y
	gravity_vector = Vector3.DOWN

	fabrik_phase1(depth)
	fabrik_phase2(depth)

	# Apply
	bones.reverse()

	for i in range(depth):
		var bone = bones[i]
		var parent_transform: Transform3D = skeleton.get_bone_global_pose(bone["parent_idx"])
		var new_transform: Transform3D = skeleton.get_bone_global_pose(bone["idx"])
		new_transform.origin = bone["from"]

		var dir = (bone["to"] - bone["from"]).normalized()
		# var prev_bone_basis = skeleton.get_bone_global_pose(bone["parent_idx"]).basis

		var ee = Basis.looking_at(dir, parent_transform.basis.z)
		new_transform.basis = Basis(ee.x.normalized(), -ee.z.normalized(), ee.y.normalized())

		skeleton.set_bone_global_pose(bone["idx"], new_transform)

func fabrik_phase1(depth: int):
	var target_pos = target.position

	for i in range(depth):
		var bone = bones[i]
		bone["to"] = target_pos

		bone["from"] = apply_bias(bone["to"], bone["from"], bone["length"], bone["parent_basis"].y, bone["stiffness"])
		bone["from"] = apply_bias(bone["to"], bone["from"], bone["length"], gravity_vector, bone["gravity"])

		target_pos = bone["from"]

func fabrik_phase2(depth: int):
	var source_pos = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).origin

	for i in range(depth):
		var bone = bones[depth - i - 1]
		bone["from"] = source_pos

		bone["to"] = apply_bias(bone["from"], bone["to"], bone["length"], bone["parent_basis"].y, bone["stiffness"])
		bone["to"] = apply_bias(bone["from"], bone["to"], bone["length"], gravity_vector, bone["gravity"])

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

func apply_bias(original_from: Vector3, original_to: Vector3, length: float, bias_to: Vector3, stiffness: float) -> Vector3:
	var desired_direction = (original_to - original_from).normalized()
	var desired_to = original_from + desired_direction * length

	var biased_to = (original_from + bias_to * length)
	var biased_position = (1 - stiffness) * desired_to + stiffness * biased_to
	var biased_direction = (biased_position - original_from).normalized()

	return original_from + biased_direction * length
