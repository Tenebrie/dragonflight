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

var gravity_vector: Vector3

@export_subgroup("Debug")
var bone_debug: IKBoneDebug
var bone_debug_enabled: bool = false
@export var editor_bone_debug: bool:
	get:
		return bone_debug_enabled
	set(value):
		bone_debug_enabled = value
		if not value and bone_debug:
			bone_debug.queue_free()
			bone_debug = null
		elif value:
			bone_debug = IKBoneDebug.new()
			add_child(bone_debug)

func _ready() -> void:
	skeleton = get_parent()

func _process(_delta: float) -> void:
	for i in range(get_child_count()):
		var child = get_child(i)
		if child.get_class() != "BoneAttachment3D" || child.bone_idx == -1:
			continue
		var bone_transform: Transform3D = skeleton.get_bone_global_pose(child.bone_idx)
		child.global_position = bone_transform.origin * skeleton.global_transform.inverse()

var d = 0
func _physics_process(_delta: float) -> void:
	d += 1
	if d % 1 != 0:
		return

	if get_child_count() == 0 or not enabled or (not preview and Engine.is_editor_hint()) or not skeleton or depth == 0:
		return

	resolve_ik(_delta)

func resolve_ik(delta: float) -> void:
	var first_child: BoneAttachment3D = get_child(1)
	var tip_bone_idx = first_child.bone_idx
	if tip_bone_idx == -1:
		return

	var root_bone_idx = find_root_bone(tip_bone_idx)

	if bone_debug:
		bone_debug.clear_mesh()

	gravity_vector = skeleton.global_transform.inverse() * Vector3.DOWN

	var source_pos = skeleton.get_bone_global_rest(root_bone_idx).origin
	var target_pos = skeleton.global_transform.inverse() * get_child(0).global_position
	
	var bones = collect_bones(tip_bone_idx)
	apply_root_rotation(bones)

	var debug_colors = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.PURPLE, Color.ORANGE, Color.PINK, Color.BROWN, Color.GRAY, Color.CYAN, Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.PURPLE, Color.ORANGE, Color.PINK, Color.BROWN, Color.GRAY, Color.CYAN, Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.PURPLE, Color.ORANGE, Color.PINK, Color.BROWN, Color.GRAY, Color.CYAN, Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.PURPLE, Color.ORANGE, Color.PINK, Color.BROWN, Color.GRAY, Color.CYAN]

	var iterations = 8
	BiasResolver.resolve(skeleton, bones, delta, iterations, gravity)

	for i in range(iterations):
		# FabrikResolver.resolve(bones, source_pos, target_pos)
		FabrikResolver.resolve_with_collision(bones, source_pos, target_pos)

		# if bone_debug:
			# bone_debug.print_bones(bones, debug_colors[i])

		if i < iterations / 2.0:
			CollisionResolver.resolve_and_land(bones)
		else:
			CollisionResolver.resolve_and_land(bones)

		bones[depth - 1].is_locked_origin = true

		if i == 7:
			FabrikResolver.reconnect_locked_bones(bones)

		for bone in bones:
			bone.is_locked_origin = false
			bone.is_locked_target = false

	if bone_debug:
		bone_debug.print_bones(bones, debug_colors[0])

	apply_bones(bones)

func find_root_bone(tip_bone_idx: int) -> int:
	var root_bone = tip_bone_idx
	for i in range(depth - 1):
		var parent = skeleton.get_bone_parent(root_bone)
		root_bone = parent
	return root_bone

func apply_root_rotation(_bones: Array[IKBone3D]):
	pass
	# TODO: Apply root rotation to the bones
	# parent_basis = parent_basis.rotated(parent_basis.z, root_yaw * PI).rotated(parent_basis.x, -root_pitch * PI)

func collect_bones(tip_bone_idx: int) -> Array[IKBone3D]:
	var bones: Array[IKBone3D] = []
	var current_bone = tip_bone_idx
	var child_idx = -1
	for i in range(depth):
		var bone = IKBone3D.make(skeleton, current_bone, child_idx)
		child_idx = bone.bone_idx
		current_bone = bone.parent_bone_idx
		bones.append(bone)
		bone.target = bone.target_forward()
		
		var bone_name = skeleton.get_bone_name(bone.bone_idx).replace(".", "_")
		var rigid_body = find_child(bone_name)
		if rigid_body:
			bone.rigid_body = rigid_body.get_child(0)

	for i in range(depth - 1):
		bones[i + 1].child_bone = bones[i]
		bones[i].parent_bone = bones[i + 1]
	return bones

class FabrikResolver:
	static func resolve(bones: Array[IKBone3D], root_pos: Vector3, target_pos: Vector3) -> void:
		_fabrik_phase1(bones, target_pos, 0, bones.size())
		_fabrik_phase2(bones, root_pos, 0, bones.size())

	static func reconnect_locked_bones(bones: Array[IKBone3D]) -> void:
		bones.reverse()
		for i in range(bones.size()):
			if i > 0:
				if not bones[i - 1].is_locked_origin:
					bones[i - 1].target = bones[i].origin
					bones[i - 1].origin = _preserve_length(bones[i - 1].target, bones[i - 1].origin, bones[i - 1].length)
				else:
					bones[i].origin = bones[i - 1].target
					bones[i].target = _preserve_length(bones[i].origin, bones[i].target, bones[i].length)
					bones[i].is_locked_origin = true
					bones[i].is_locked_target = true
		bones.reverse()

	static func resolve_with_collision(bones: Array[IKBone3D], root_pos: Vector3, target_pos: Vector3) -> void:
		_fabrik_phase1_collision_aware(bones, target_pos, 0, bones.size())
		_fabrik_phase2_collision_aware(bones, root_pos, 0, bones.size())

		# _fabrik_phase1(bones, target_pos, 0, bones.size())
		# _fabrik_phase2(bones, root_pos, 0, bones.size())

	static func _fabrik_phase1(bones: Array[IKBone3D], target_pos: Vector3, from_index: int, to_index: int):
		for i in range(from_index, to_index):
			var bone = bones[i]

			bone.target = target_pos
			bone.origin = _preserve_length(bone.target, bone.origin, bone.length)

			target_pos = bone.origin

	static func _fabrik_phase2(bones: Array[IKBone3D], source_pos: Vector3, from_index: int, to_index: int):
		for i in range(to_index - 1, from_index - 1, -1):
			var bone = bones[i]

			bone.origin = source_pos
			bone.target = _preserve_length(bone.origin, bone.target, bone.length)

			source_pos = bone.target

	static func _fabrik_phase1_collision_aware(bones: Array[IKBone3D], target_pos: Vector3, from_index: int, to_index: int):
		for i in range(from_index, to_index):
			var bone = bones[i]

			# bone.target = IKUtils.move_and_collide(bone.skeleton, bone.target, bone.target, target_pos).point
			var desired_origin = _preserve_length(target_pos, bone.origin, bone.length)
			var line_move_data = IKUtils.line_move_and_collide(bone.skeleton, bone.origin, bone.target, desired_origin, target_pos, bone, true)
			bone.origin = _preserve_length(line_move_data.to, line_move_data.from, bone.length)
			bone.target = line_move_data.to

			target_pos = bone.origin

	static func _fabrik_phase2_collision_aware(bones: Array[IKBone3D], source_pos: Vector3, from_index: int, to_index: int):
		for i in range(to_index - 1, from_index - 1, -1):
			var bone = bones[i]

			var desired_target = _preserve_length(source_pos, bone.target, bone.length)
			var line_move_data = IKUtils.line_move_and_collide(bone.skeleton, bone.origin, bone.target, source_pos, desired_target, bone, true)
			bone.origin = line_move_data.from
			bone.target = _preserve_length(line_move_data.from, line_move_data.to, bone.length)

			source_pos = bone.target


	static func _preserve_length(original_from: Vector3, original_to: Vector3, length: float) -> Vector3:
		var dir = (original_to - original_from).normalized()
		var target = original_from + dir * length

		return target

class BiasResolver:
	static func resolve(skeleton: Skeleton3D, bones: Array[IKBone3D], _delta: float, iterations: int, gravity: float) -> void:
		var gravity_this_iteration = 9.8 * gravity
		var gravity_vector = (skeleton.global_transform.inverse().basis * Vector3.DOWN).normalized()
		for i in range(bones.size()):
			var bone = bones[i]
			bone.origin = bone.origin + gravity_this_iteration * gravity_vector * _delta
			bone.target = bone.target + gravity_this_iteration * gravity_vector * _delta

class CollisionResolver:
	static func resolve_and_push(bones: Array[IKBone3D]) -> void:
		bones.reverse()
		for bone in bones:
			_resolve_bone(bone, false)
		bones.reverse()

	static func resolve_and_land(bones: Array[IKBone3D]) -> void:
		bones.reverse()
		for bone in bones:
			_resolve_bone(bone, true)
		bones.reverse()

	static func _resolve_bone(desired: IKBone3D, land: bool) -> void:
		var current = desired.skeleton.get_bone_global_pose(desired.bone_idx)
		var current_target = current.origin + current.basis.y * desired.length

		var line_move_data = IKUtils.line_move_and_collide(desired.skeleton, current.origin, current_target, desired.origin, desired.target, desired, land)
		desired.origin = line_move_data.from
		desired.target = line_move_data.to
		# if line_move_data.is_hit:
			# desired.is_locked_origin = true
			# desired.is_locked_target = true

	static func _reconnect_bone(bone: IKBone3D, target_pos: Vector3) -> void:
		if not bone.is_locked_target and not bone.is_locked_origin:
			var looking_at: Vector3
			if bone.child_bone:
				looking_at = bone.child_bone.origin
			else:
				looking_at = target_pos

			var current = bone.skeleton.get_bone_global_pose(bone.bone_idx)
			var forward = current.basis.y
			var new_forward = (looking_at - bone.origin).normalized()

			var target = bone.origin + forward * bone.length
			var new_target = bone.origin + new_forward * bone.length

			var solved_data = IKUtils.move_and_collide(bone.skeleton, bone.origin, target, new_target)

			var solved_target = solved_data.point
			# if not bone.is_locked_target:
				# bone.target = solved_target
			# if bone.child_bone and not bone.child_bone.is_locked_origin:
				# bone.child_bone.origin = bone.target

		# if bone.is_locked_origin && bone.parent_bone:
			# bone.parent_bone.target = bone.origin

class SmoothingResolver:
	static func smooth_joint(bones: Array[IKBone3D], i: int, weight := 0.5) -> void:
		if i == 0 or i == bones.size() - 1:
			return

		var prev = bones[i - 1].origin
		var next = bones[i + 1].origin
		var midpoint = (prev + next) * 0.5
		bones[i].origin = bones[i].origin.lerp(midpoint, weight)

	static func resolve(bones: Array[IKBone3D]) -> void:
		for i in range(1, bones.size() - 1):
			smooth_joint(bones, i)

func apply_bones(bones: Array[IKBone3D]):
	# Expects bones tip-to-root, reversing the order of application.
	bones.reverse()

	for bone in bones:
		var data = IKBone3D.BoneFlushData.from_bone(bone)
		bone.flush_changes(bone.flush_prepare(data))
	
	bones.reverse()

func apply_bias(original_from: Vector3, original_to: Vector3, bias_to: Vector3, influence: float) -> Vector3:
	var biased_to = (original_to + bias_to * influence)

	return IKUtils.move_and_collide(skeleton, original_from, original_to, biased_to).point

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
