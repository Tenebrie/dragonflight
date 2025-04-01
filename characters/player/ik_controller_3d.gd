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

var myvar = 0.0

func _physics_process(_delta: float) -> void:
	if myvar == null:
		myvar = 0.0

	if not preview:
		return
	if not target or not target.is_inside_tree():
		return

	myvar += _delta / 10

	var depth = 14
	var bones = []
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
			"roll": 0.0
		})
		current_bone = parent
		current_to = from

	var root_up = - skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).basis.z
	var root_forward = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).basis.y

	var source_pos = bones[depth - 1]["from"]
	var target_pos = target_position

	# Phase 1
	for i in range(depth):
		var bone = bones[i]
		bone["to"] = target_pos
		var direction = (bone["to"] - bone["from"]).normalized()
		bone["from"] = bone["to"] - direction * bone["length"]
		
		target_pos = bone["from"]

	# Phase 2
	var up_vector = root_up
	var forward_vector = root_forward
	for i in range(depth):
		var bone = bones[depth - i - 1]
		bone["from"] = source_pos
		var desired_direction: Vector3 = (bone["to"] - bone["from"]).normalized()

		var normal = forward_vector.cross(up_vector).normalized()

		var yaw = atan2(desired_direction.x, desired_direction.y) # angle in forward/normal plane
		var pitch = atan2(desired_direction.z, desired_direction.y) # angle in forward/up plane
		var roll = atan2(desired_direction.x, desired_direction.z) # angle in up/normal plane

		yaw = angle_clampf(yaw, PI / 3)
		pitch = angle_clampf(pitch, PI / 3)
		roll = angle_clampf(roll, 0)

		# var pole_vector = Vector3.ZERO
		# if pole_target:
			# pole_vector = (pole_target.global_position - bone["from"]).normalized()

		var mybasis = Basis().rotated(up_vector, PI - yaw).rotated(normal, PI - pitch).rotated(forward_vector, PI - roll)
		# var mybasis = Basis().rotated(up_vector, PI - yaw)

		# if pole_target:
			# var pole_angle = desired_direction.angle_to(pole_vector)
			# var pole_rotation = desired_direction.cross(pole_vector).normalized()
			# mybasis = mybasis.rotated(pole_rotation, pole_angle * 0)

		var reconstructed = (forward_vector * mybasis).normalized()

		desired_direction = reconstructed
		bone["pitch"] = PI - pitch
		bone["roll"] = 0
		bone["yaw"] = PI - yaw

		bone["to"] = bone["from"] + desired_direction * bone["length"]

		# Don't touch, I guess?
		# forward_vector = forward_vector.rotated(normal, PI - pitch)
		# up_vector = up_vector.rotated(normal, PI - pitch)

		source_pos = bone["to"]

	# print(bones)

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
		var g = skeleton.get_bone_global_pose(bone["idx"])
		g.origin = bone["from"]
		g.basis = Basis().rotated(Vector3.RIGHT, bone["pitch"]).rotated(Vector3.FORWARD, bone["yaw"]).rotated(Vector3.UP, bone["roll"])
		skeleton.set_bone_global_pose(bone["idx"], g)

func angle_clampf(value: float, limit: float) -> float:
	# TODO: It only works for reversed angles now, fix pls
	if value > 0:
		return maxf(value, PI - limit)
	else:
		return minf(value, -PI + limit)
