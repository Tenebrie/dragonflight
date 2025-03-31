@tool

extends BoneAttachment3D

@export var target: Node3D
@export var skeleton: Skeleton3D

var initial_rotation: Vector3

var preview: bool = false
@export var enable_preview: bool:
	get:
		return preview
	set(value):
		preview = value

func _ready() -> void:
	initial_rotation = rotation_degrees


func _physics_process(_delta: float) -> void:
	if not preview:
		return
	if not target or not target.is_inside_tree():
		return

	var depth = 6
	var bones = []
	var current_bone = bone_idx

	var current_to = target.position
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
			"length": length
		})
		current_bone = parent
		current_to = from

	var source_pos = bones[depth - 1]["from"]
	var target_pos = target.position

	# Phase 1
	for i in range(depth):
		var bone = bones[i]
		bone["to"] = target_pos
		var direction = (bone["to"] - bone["from"]).normalized()
		bone["from"] = bone["to"] - direction * bone["length"]
		target_pos = bone["from"]


	# Phase 2
	for i in range(depth):
		var bone = bones[depth - i - 1]
		bone["from"] = source_pos
		var direction = (bone["to"] - bone["from"]).normalized()
		bone["to"] = bone["from"] + direction * bone["length"]
		source_pos = bone["to"]

	# print(bones)

	# Apply
	bones.reverse()
	for i in range(depth):
		var bone = bones[i]
		var g = skeleton.get_bone_global_pose(bone["idx"])
		g.origin = bone["from"]
		skeleton.set_bone_global_pose(bone["idx"], g)
		
		var direction = (bone["to"] - bone["from"]).normalized()
		
		var up = skeleton.get_bone_global_pose(skeleton.get_bone_parent(bone["idx"])).basis.z
		
		# Project up vector onto the plane perpendicular to direction
		up = (up - direction * direction.dot(up)).normalized()
		
		var look_at_basis = Basis.looking_at(direction, up)
		var parent_basis = skeleton.get_bone_global_pose(bone["parent_idx"]).basis
		var local_basis = parent_basis.inverse() * look_at_basis
		# Adjust the initial rotation offset based on your bone's rest pose
		local_basis *= Basis().rotated(Vector3.RIGHT, -PI / 2)
		var constrained = Basis.from_euler(apply_rotation_constraint(local_basis.get_euler()))
		
		var quart = constrained.get_rotation_quaternion()
		skeleton.set_bone_pose_rotation(bone["idx"], quart)

func apply_rotation_constraint(angle: Vector3) -> Vector3:
	return angle