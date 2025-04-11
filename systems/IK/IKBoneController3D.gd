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

func _physics_process(_delta: float) -> void:
	if get_child_count() == 0 or not enabled or (not preview and Engine.is_editor_hint()) or not skeleton or depth == 0:
		return

	resolve_ik(_delta)

func resolve_ik(_delta: float) -> void:
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

	var debug_colors = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.PURPLE, Color.ORANGE, Color.PINK, Color.BROWN, Color.GRAY, Color.CYAN]

	resolve_fabrik(bones, source_pos, target_pos)

	if bone_debug:
		bone_debug.print_bones(bones, debug_colors[0])

	# Biases
	# var desired_from = bone["from"]
	# desired_from = apply_bias(bone["to"], desired_from, bone["length"], bone["parent_basis"].y, bone["stiffness"])
	# desired_from = apply_bias(bone["to"], desired_from, bone["length"], gravity_vector, bone["gravity"])

	apply_bones(bones)

func find_root_bone(tip_bone_idx: int) -> int:
	var root_bone = tip_bone_idx
	for i in range(depth):
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
	return bones

func resolve_fabrik(bones: Array[IKBone3D], root_pos: Vector3, target_pos: Vector3) -> void:
	fabrik_phase1(bones, target_pos, 0, depth)
	fabrik_phase2(bones, root_pos, 0, depth)

func fabrik_phase1(bones: Array[IKBone3D], target_pos: Vector3, from_index: int, to_index: int):
	for i in range(from_index, to_index):
		var bone = bones[i]

		bone.target = target_pos
		bone.origin = preserve_length(bone.target, bone.origin, bone.length)

		target_pos = bone.origin

func fabrik_phase2(bones: Array[IKBone3D], source_pos: Vector3, from_index: int, to_index: int):
	for i in range(to_index - 1, from_index - 1, -1):
		var bone = bones[i]

		bone.origin = source_pos
		bone.target = preserve_length(bone.origin, bone.target, bone.length)

		source_pos = bone.target

func apply_bones(bones: Array[IKBone3D]):
	# Expects bones tip-to-root, reversing the order of application.
	bones.reverse()

	for bone in bones:
		var data = IKBone3D.BoneFlushData.from_bone(bone)
		bone.flush_changes(bone.flush_prepare(data))
	
	bones.reverse()

func preserve_length(original_from: Vector3, original_to: Vector3, length: float, stretch: float = 0.0) -> Vector3:
	var dir = (original_to - original_from).normalized()
	var target = original_from + dir * length

	return target.lerp(original_to, target.distance_to(original_to) * stretch)

func apply_bias(original_from: Vector3, original_to: Vector3, bias_to: Vector3, influence: float) -> Vector3:
	var biased_to = (original_to + bias_to * influence)

	return move_and_collide(original_from, original_to, biased_to)

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

func move_and_collide(sample_origin: Vector3, from: Vector3, to: Vector3) -> Vector3:
	var data = get_safe_point_for_line(sample_origin, from, to)

	return data["point"]

func get_safe_point_for_line(skeleton_sample_origin: Vector3, skeleton_from: Vector3, skeleton_to: Vector3) -> Dictionary:
	var sample_count = 10 # Increased sample count for better precision
	
	var sample_origin = skeleton.global_transform * skeleton_sample_origin
	var sample_start = skeleton.global_transform * skeleton_from
	var sample_end = skeleton.global_transform * skeleton_to
	
	# First check if we can move directly to the target
	var initial_movement_query = PhysicsRayQueryParameters3D.new()
	initial_movement_query.from = sample_start
	initial_movement_query.to = sample_end
	var initial_movement_result = get_world_3d().direct_space_state.intersect_ray(initial_movement_query)
	if initial_movement_result.size() == 0:
		# If we can move directly, check if the line from origin to target is clear
		var origin_query = PhysicsRayQueryParameters3D.new()
		origin_query.from = sample_origin
		origin_query.to = sample_end
		if get_world_3d().direct_space_state.intersect_ray(origin_query).size() == 0:
			return {
				"point": skeleton_to,
				"resolved": true
			}
	
	# If direct movement is blocked, try to find a safe path
	var safe_point_found = false
	var last_known_safe_point = null
	var closest_to_target = null
	var closest_distance = INF
	
	for i in range(sample_count + 1):
		var t = float(i) / sample_count
		var current_point = sample_start.lerp(sample_end, t)
		
		# First check if we can move to this point
		var current_movement_query = PhysicsRayQueryParameters3D.new()
		current_movement_query.from = sample_start
		current_movement_query.to = current_point
		var current_movement_result = get_world_3d().direct_space_state.intersect_ray(current_movement_query)
		if current_movement_result.size() > 0:
			continue # Skip this point if we can't move to it
		
		# Then check if the line from origin to this point is clear
		var origin_query = PhysicsRayQueryParameters3D.new()
		origin_query.from = sample_origin
		origin_query.to = current_point
		var origin_result = get_world_3d().direct_space_state.intersect_ray(origin_query)
		if origin_result.size() > 0:
			if safe_point_found:
				# We found a collision after having a safe point
				return {
					"point": skeleton.global_transform.inverse() * last_known_safe_point,
					"resolved": true
				}
			# Try to find a safe direction around the obstacle
			var d = (sample_end - sample_origin).normalized()
			var dot = d.dot(origin_result.normal)
			var safe_direction = (d - origin_result.normal * dot).normalized()
			
			# Try multiple angles around the obstacle
			var best_angle = 0.0
			var best_distance = INF

			var angles_to_try = 8
			var angle_step = PI / angles_to_try
			
			for angle in range(angles_to_try):
				var rotated_direction = safe_direction.rotated(origin_result.normal, angle * angle_step)
				var test_point = sample_origin + rotated_direction * sample_origin.distance_to(sample_end)
				
				# Check if we can move to this point
				var test_movement_query = PhysicsRayQueryParameters3D.new()
				test_movement_query.from = sample_start
				test_movement_query.to = test_point
				if get_world_3d().direct_space_state.intersect_ray(test_movement_query).size() > 0:
					continue
				
				# Check if the line from origin to this point is clear
				var test_origin_query = PhysicsRayQueryParameters3D.new()
				test_origin_query.from = sample_origin
				test_origin_query.to = test_point
				if get_world_3d().direct_space_state.intersect_ray(test_origin_query).size() == 0:
					var distance = test_point.distance_to(sample_end)
					if distance < best_distance:
						best_distance = distance
						best_angle = angle * angle_step
			
			if best_distance < INF:
				var final_direction = safe_direction.rotated(origin_result.normal, best_angle)
				var final_point = sample_origin + final_direction * sample_origin.distance_to(sample_end)
				
				return get_safe_point_for_line(
					skeleton_sample_origin,
					skeleton.global_transform.inverse() * final_point,
					skeleton_to,
				)
		
		safe_point_found = true
		last_known_safe_point = current_point
		
		# Keep track of the point closest to the target
		var distance_to_target = current_point.distance_to(sample_end)
		if distance_to_target < closest_distance:
			closest_distance = distance_to_target
			closest_to_target = current_point
	
	if safe_point_found:
		return {
			"point": skeleton.global_transform.inverse() * last_known_safe_point,
			"resolved": true
		}
	
	# If we couldn't find a safe point, return the closest point to the target
	if closest_to_target:
		return {
			"point": skeleton.global_transform.inverse() * closest_to_target,
			"resolved": true
		}
	
	# Last resort: return the starting point
	return {
		"point": skeleton_from,
		"resolved": false
	}
