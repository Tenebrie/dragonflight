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
var bias_vector: Vector3

func _physics_process(_delta: float) -> void:
	if not preview:
		return
	if not target or not target.is_inside_tree():
		return

	var depth = 14
	bones = []
	var current_bone = bone_idx

	# var orr = skeleton.get_bone_global_pose(bone_idx).origin
	# var target_position = lerp(orr, target.position, 0.5 * _delta)
	var target_position = target.position
	var current_to = target_position

	for i in range(depth):
		var parent = skeleton.get_bone_parent(current_bone)
		
		var from = skeleton.get_bone_global_pose(current_bone).origin
		var to = current_to
		# If it's the first bone, length is not relevant
		var length = 1.0 if bones.is_empty() else skeleton.get_bone_global_rest(current_bone).origin.distance_to(skeleton.get_bone_global_rest(parent).origin)
		bones.append({
			"idx": current_bone,
			"parent_idx": parent,
			"from": from,
			"to": to,
			"length": length,
			"pitch": 0.0,
			"yaw": 0.0,
			"roll": 0.0,
			"stiffness": 0.0,
			"rest_basis": skeleton.get_bone_global_pose(parent).basis
		})
		current_bone = parent
		current_to = from

	root_up = Vector3(0, 0, -1)
	root_forward = Vector3(0, -1, 0)
	# root_up = -skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).basis.z
	# root_forward = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).basis.y
	bias_vector = -root_up

	fabrik_phase1(depth)
	fabrik_phase2(depth)

	# Apply
	bones.reverse()

	# Bake in rotations
	var pitch_acc = PI
	var yaw_acc = 0.0
	var roll_acc = PI
	for i in range(depth):
		var bone = bones[i]
		pitch_acc = bone["pitch"] + PI
		yaw_acc = bone["yaw"]
		roll_acc = bone["roll"] + PI
		bone["pitch"] = pitch_acc
		bone["yaw"] = yaw_acc
		bone["roll"] = roll_acc

	for i in range(depth):
		var bone = bones[i]
		var new_transform: Transform3D = skeleton.get_bone_global_pose(bone["idx"])
		new_transform.origin = bone["from"]
		new_transform.basis = Basis().rotated(Vector3.RIGHT, bone["pitch"]).rotated(Vector3.FORWARD, bone["yaw"]).rotated(Vector3.UP, bone["roll"])

		skeleton.set_bone_global_pose(bone["idx"], new_transform)

func fabrik_phase1(depth: int):
	var up_vector = root_up
	var forward_vector = root_forward

	var target_pos = target.position
	for i in range(depth):
		var bone = bones[i]
		bone["to"] = target_pos
		var desired_direction: Vector3 = (bone["to"] - bone["from"]).normalized()

		var constrained = apply_rotation_constraints(bone["rest_basis"], up_vector, forward_vector, desired_direction)
		desired_direction = constrained["direction"]
		bone["pitch"] = constrained["pitch"]
		bone["roll"] = constrained["roll"]
		bone["yaw"] = constrained["yaw"]

		bone["to"] = apply_bias(bone["from"], desired_direction, bone["length"], bone["stiffness"])
		
		target_pos = bone["to"]

func fabrik_phase2(depth: int):
	var up_vector = root_up
	var forward_vector = root_forward

	var source_pos = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).origin

	for i in range(depth):
		var bone = bones[depth - i - 1]
		bone["from"] = source_pos
		var desired_direction: Vector3 = (bone["to"] - bone["from"]).normalized()

		var constrained = apply_rotation_constraints(bone["rest_basis"], up_vector, forward_vector, desired_direction)
		desired_direction = constrained["direction"]
		bone["pitch"] = constrained["pitch"]
		bone["roll"] = constrained["roll"]
		bone["yaw"] = constrained["yaw"]

		bone["to"] = apply_bias(bone["from"], desired_direction, bone["length"], bone["stiffness"])

		source_pos = bone["to"]

func apply_rotation_constraints(rest_basis: Basis, up_vector: Vector3, forward_vector: Vector3, desired_direction: Vector3):
	var normal = forward_vector.cross(up_vector).normalized()

	var yaw = atan2(desired_direction.x, desired_direction.y) # angle in forward/normal plane
	var pitch = atan2(desired_direction.z, desired_direction.y) # angle in forward/up plane
	var roll = atan2(desired_direction.x, desired_direction.z) # angle in up/normal plane

	var rest_forward = rest_basis.y

	var rest_yaw = atan2(rest_forward.x, rest_forward.y) # angle in forward/normal plane
	var rest_pitch = atan2(rest_forward.z, rest_forward.y) # angle in forward/up plane
	var rest_roll = atan2(desired_direction.x, desired_direction.z) # angle in up/normal plane

	var yaw_diff = rest_yaw - yaw
	if yaw_diff >= PI:
		yaw_diff = -2 * PI + yaw_diff
	elif yaw_diff <= -PI:
		yaw_diff = 2 * PI + yaw_diff

	var pitch_diff = rest_pitch - pitch
	if pitch_diff >= PI:
		pitch_diff = -2 * PI + pitch_diff
	elif pitch_diff <= -PI:
		pitch_diff = 2 * PI + pitch_diff

	var roll_diff = rest_roll - roll
	if roll_diff >= PI:
		roll_diff = -2 * PI + roll_diff
	elif roll_diff <= -PI:
		roll_diff = 2 * PI + roll_diff

	var limit = PI / 16
	yaw_diff = clampf(yaw_diff, -limit, limit)
	pitch_diff = clampf(pitch_diff, -limit, limit)
	roll_diff = clampf(roll_diff, -limit, limit)

	yaw = rest_yaw - yaw_diff
	pitch = rest_pitch - pitch_diff
	roll = rest_roll - roll_diff

	var mybasis = Basis().rotated(up_vector, PI - yaw).rotated(normal, PI - pitch).rotated(forward_vector, PI - roll)
	return {
		direction = (forward_vector * mybasis).normalized(),
		pitch = PI - pitch,
		roll = 0,
		yaw = PI - yaw
	}

func apply_bias(original_from: Vector3, desired_direction: Vector3, length: float, stiffness: float) -> Vector3:
	var desired_to = original_from + desired_direction * length

	var biased_to = (original_from + bias_vector * length)
	var biased_position = (1 - stiffness) * desired_to + stiffness * biased_to
	var biased_direction = (biased_position - original_from).normalized()

	return original_from + biased_direction * length

func angle_clampf(value: float, limit: float) -> float:
	# TODO: It only works for reversed angles now, fix pls
	if value > 0:
		return maxf(value, PI - limit)
	else:
		return minf(value, -PI + limit)
