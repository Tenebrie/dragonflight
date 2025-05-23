extends BoneAttachment3D
class_name IKBoneController3D
## Fabrik-based inverse kinematics for 3D bones

@export var target: Node3D
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

@export_subgroup("Jolt Integration")
@export var simulator: PhysicalBoneSimulator3D

var skeleton: Skeleton3D
var bones: Array[Dictionary] = []

var gravity_vector: Vector3

func _ready() -> void:
	skeleton = get_parent()

func _physics_process(_delta: float) -> void:
	if not enabled:
		return
	if not preview and Engine.is_editor_hint():
		return
	if not target or not target.is_inside_tree():
		return
	if not skeleton:
		return
	if bone_idx == -1:
		return
	if depth == 0:
		return

	bones = []
	var current_bone = bone_idx

	var current_to = target.position

	# var log = EditorInterface.get_editor_main_screen().find_child("EditorLog")
	# print(log)

	# TODO: No need to recalculate everything every frame
	for i in range(depth):
		var parent = skeleton.get_bone_parent(current_bone)
		if parent == -1:
			push_warning("[IKBoneController3D] Bone '%s' (depth = %d) has no parent. Depth too high?" % [skeleton.get_bone_name(current_bone), i])
			return
		
		var from = skeleton.get_bone_global_pose(current_bone).origin
		var to = current_to
		# If it's the first bone, length is not available
		var length = 0.1 if bones.is_empty() else to.distance_to(from)
		var parent_basis = skeleton.get_bone_global_pose(parent).basis
		var rest_basis = skeleton.get_bone_global_rest(current_bone).basis

		if i == depth - 1:
			parent_basis = parent_basis.rotated(parent_basis.z, root_yaw * PI).rotated(parent_basis.x, -root_pitch * PI)

		var physical_bone: PhysicalBone3D
		var desired_from = from

		if simulator:
			var physical_bone_name = "Physical Bone " + skeleton.get_bone_name(current_bone).replace(".", "_")
			physical_bone = simulator.find_child(physical_bone_name)
			if physical_bone:
				physical_bone.mass = 10
				physical_bone.custom_integrator = false
				var temp = physical_bone.global_position + physical_bone.joint_offset.origin * physical_bone.basis.inverse()
				var final = temp * skeleton.global_transform

				from = final

		var bone = {
			"idx": current_bone,
			"parent_idx": parent,
			"desired_from": desired_from,
			"from": from,
			"to": to,
			"length": length,
			"gravity": clamp(gravity + bone_weight * i, 0, 1),
			"stiffness": clamp(stiffness + stiffness_per_bone * i, 0, 1),
			# "gravity": -1,
			# "stiffness": 0.2 + 0.4 * i,
			"parent_basis": parent_basis,
			"rest_basis": rest_basis,
			"physical_bone": physical_bone,
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
		# var preferred_up = old_up

		var twist = get_roll_difference(new_transform, parent_transform)

		var untwisted_up = preferred_up.rotated(forward, twist)
		preferred_up = preferred_up.lerp(untwisted_up, 1)

		preferred_up = old_up.lerp(preferred_up, 1 - rotational_stiffness)
	
		var up = forward.cross(preferred_up).normalized()
		var normal = forward.cross(up).normalized()
		
		new_transform.basis = Basis(normal, forward, up).orthonormalized()

		if bone["physical_bone"]:
			var physical_bone: IKPhysicalBone3D = bone["physical_bone"]

			var temp = physical_bone.global_position + physical_bone.joint_offset.origin * physical_bone.basis.inverse()
			var bone_pivot = temp
			var target_point = skeleton.global_transform * bone["to"]

			var physical_forward = - physical_bone.transform.basis.z
			var desired_forward = forward
			# var desired_dir: Vector3 = (target_point - bone_pivot).normalized()

			var dir_to_target = (target_point - bone_pivot)
			var magnitude = dir_to_target.length() * _delta * 350
			var direction = dir_to_target.normalized()
			var final = direction * magnitude
			# physical_bone.global_position = actual_from
			# physical_bone.global_transform = new_transform
			physical_bone.rest_forward = (skeleton.global_transform * skeleton.get_bone_global_rest(bone["idx"])).basis.y
			physical_bone.apply_impulse(final)
			# PhysicsServer3D.body_add_constant_force(phys_bone.get_rid(), Vector3.UP)
		
		# skeleton.set_bone_global_pose(bone["idx"], new_transform)
			

func fabrik_phase1():
	var target_pos = target.position

	for i in range(depth):
		var bone = bones[i]
		bone["to"] = target_pos

		bone["from"] = apply_bias(bone["to"], bone["from"], bone["length"], bone["parent_basis"].y, bone["stiffness"])
		bone["from"] = apply_bias(bone["to"], bone["from"], bone["length"], -gravity_vector, bone["gravity"])

		target_pos = bone["from"]

func fabrik_phase2():
	var source_pos = skeleton.get_bone_global_rest(bones[depth - 1]["idx"]).origin

	for i in range(depth):
		var bone = bones[depth - i - 1]
		bone["from"] = source_pos

		bone["to"] = apply_bias(bone["from"], bone["to"], bone["length"], bone["parent_basis"].y, bone["stiffness"])
		bone["to"] = apply_bias(bone["from"], bone["to"], bone["length"], -gravity_vector, bone["gravity"])

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
