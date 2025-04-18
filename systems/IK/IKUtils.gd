class_name IKUtils

# Returns the roll difference (in radians) between transform1 and transform2,
# measured around the provided twist_axis.
static func get_roll_difference(transform1: Transform3D, transform2: Transform3D) -> float:
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

# TODO: Rotational constraints
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

class CollisionData:
	var is_hit: bool
	var is_resolved: bool
	var point: Vector3

	static func make(is_hit: bool, is_resolved: bool, point: Vector3) -> CollisionData:
		var data = CollisionData.new()
		data.is_hit = is_hit
		data.is_resolved = is_resolved
		data.point = point
		return data

class LineCollisionData:
	var is_hit: bool
	var is_resolved: bool
	var from: Vector3
	var to: Vector3

	static func make(is_hit: bool, is_resolved: bool, from: Vector3, to: Vector3) -> LineCollisionData:
		var data = LineCollisionData.new()
		data.is_hit = is_hit
		data.is_resolved = is_resolved
		data.from = from
		data.to = to
		return data

static func line_move_and_collide(
	skeleton: Skeleton3D,
	original_from: Vector3,
	original_to: Vector3,
	target_from: Vector3,
	target_to: Vector3,
	data_bone: IKBone3D,
	land: bool
) -> LineCollisionData:
	var data = _line_move_and_collide(
		skeleton,
		skeleton.global_transform * original_from,
		skeleton.global_transform * original_to,
		skeleton.global_transform * target_from,
		skeleton.global_transform * target_to,
		data_bone,
		land
	)

	data.from = skeleton.global_transform.inverse() * data.from
	data.to = skeleton.global_transform.inverse() * data.to

	return data

static func _line_move_and_collide(
	skeleton: Skeleton3D,
	original_from: Vector3,
	original_to: Vector3,
	target_from: Vector3,
	target_to: Vector3,
	data_bone: IKBone3D,
	land: bool
) -> LineCollisionData:
	if not data_bone.rigid_body:
		return LineCollisionData.make(false, true, original_from, original_to)

	var sample_count = 15

	var has_collision = false
	var safe_point_found = false
	var safe_from = original_from
	var safe_to = original_to

	var length = original_from.distance_to(original_to)

	for i in range(sample_count):
		var query = PhysicsShapeQueryParameters3D.new()
		var shape = data_bone.rigid_body.get_child(0).shape
		query.shape = shape

		var current_transform = data_bone.rigid_body.get_child(0).global_transform
		var target_transform = Transform3D()
		var temp_basis = Basis.looking_at(target_to - target_from, Vector3.UP)

		target_transform.basis = Basis(temp_basis.x, -temp_basis.z, temp_basis.y)

		var shape_offset = target_transform.basis * data_bone.rigid_body.get_child(0).transform.origin
		target_transform.origin = target_from + shape_offset
		# print(target_transform.basis.y * 0.5, " ", shape_offset)

		query.transform = current_transform.interpolate_with(target_transform, float(i + 1) / sample_count)

		var result = skeleton.get_world_3d().direct_space_state.get_rest_info(query)

		var from = query.transform.origin - query.transform.basis.y * length / 2
		var to = query.transform.origin + query.transform.basis.y * length / 2

		if result.size() == 0:
			safe_point_found = true
			safe_from = from
			safe_to = to
			continue

		var normal = result.normal
		var distance = distance_to_line(from, to, result.point)
		var angle_from_a = (result.point - from).angle_to(to - from)
		if angle_from_a > PI / 2:
			distance = result.point.distance_to(from)
			normal = (from - result.point).normalized()

		var angle_from_b = (result.point - to).angle_to(from - to)
		if angle_from_b > PI / 2:
			distance = result.point.distance_to(to)
			normal = (to - result.point).normalized()

		if distance < 0.0501:
			var overshoot = 0.05 - distance
			from = from + normal * overshoot
			to = to + normal * overshoot
			return LineCollisionData.make(true, true, from, to)

		has_collision = false
		safe_point_found = true
		safe_from = from
		safe_to = to
		continue

	return LineCollisionData.make(has_collision, safe_point_found, safe_from, safe_to)


static func distance_to_line(base_a: Vector3, base_b: Vector3, point: Vector3) -> float:
	var line_vector = base_b - base_a
	var point_to_a = point - base_a
	var cross_product = point_to_a.cross(line_vector)
	var distance = cross_product.length() / line_vector.length()
	return distance

static func move_and_collide(skeleton: Skeleton3D, sample_origin: Vector3, from: Vector3, to: Vector3) -> CollisionData:
	var data = get_safe_point(
		skeleton,
		skeleton.global_transform * sample_origin,
		skeleton.global_transform * from,
		skeleton.global_transform * to
	)

	data.point = skeleton.global_transform.inverse() * data.point

	return data

static func move_and_slide(skeleton: Skeleton3D, sample_origin: Vector3, from: Vector3, to: Vector3) -> CollisionData:
	var data = _get_safe_point_for_line(
		skeleton,
		skeleton.global_transform * sample_origin,
		skeleton.global_transform * from,
		skeleton.global_transform * to
	)

	data.point = skeleton.global_transform.inverse() * data.point

	return data

static func get_safe_point(skeleton: Skeleton3D, sample_origin: Vector3, from: Vector3, to: Vector3) -> CollisionData:
	var sample_count = 10

	var query = PhysicsRayQueryParameters3D.new()
	
	var is_safe_point_found = false
	var safe_point = from

	for i in range(sample_count + 1):
		query.from = sample_origin
		query.to = from.lerp(to, float(i) / sample_count)
		var result = skeleton.get_world_3d().direct_space_state.intersect_ray(query)

		if result.size() == 0:
			is_safe_point_found = true
			safe_point = query.to
			continue

		if is_safe_point_found:
			return CollisionData.make(true, true, safe_point)

	if is_safe_point_found:
		return CollisionData.make(false, true, safe_point)

	# Unable to find a clear path, mark it as non-resolved
	return CollisionData.make(true, false, safe_point)

static func _get_safe_point_for_line(skeleton: Skeleton3D, sample_origin: Vector3, sample_start: Vector3, sample_end: Vector3, skip_secondary: bool = false) -> CollisionData:
	var sample_count = 10 # Increased sample count for better precision
	
	# First check if we can move directly to the target
	var initial_movement_query = PhysicsRayQueryParameters3D.new()
	initial_movement_query.from = sample_start
	initial_movement_query.to = sample_end
	var initial_movement_result = skeleton.get_world_3d().direct_space_state.intersect_ray(initial_movement_query)
	if initial_movement_result.size() == 0:
		# If we can move directly, check if the line from origin to target is clear
		var origin_query = PhysicsRayQueryParameters3D.new()
		origin_query.from = sample_origin
		origin_query.to = sample_end
		if skeleton.get_world_3d().direct_space_state.intersect_ray(origin_query).size() == 0:
			return CollisionData.make(false, true, sample_end)
	
	
	# If direct movement is blocked, try to find a safe path
	var safe_point_found = false
	var last_known_safe_point = null
	
	for i in range(sample_count + 1):
		var t = float(i) / sample_count
		var current_point = sample_start.lerp(sample_end, t)
		
		# First check if we can move to this point
		var current_movement_query = PhysicsRayQueryParameters3D.new()
		current_movement_query.from = sample_start
		current_movement_query.to = current_point
		var current_movement_result = skeleton.get_world_3d().direct_space_state.intersect_ray(current_movement_query)
		if current_movement_result.size() > 0:
			continue # Skip this point if we can't move to it
		
		# Then check if the line from origin to this point is clear
		var origin_query = PhysicsRayQueryParameters3D.new()
		origin_query.from = sample_origin
		origin_query.to = current_point
		var origin_result = skeleton.get_world_3d().direct_space_state.intersect_ray(origin_query)
		if origin_result.size() > 0:
			if safe_point_found:
				# We found a collision after having a safe point
				return CollisionData.make(true, true, last_known_safe_point)

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
				if skeleton.get_world_3d().direct_space_state.intersect_ray(test_movement_query).size() > 0:
					continue
				
				# Check if the line from origin to this point is clear
				var test_origin_query = PhysicsRayQueryParameters3D.new()
				test_origin_query.from = sample_origin
				test_origin_query.to = test_point
				if skeleton.get_world_3d().direct_space_state.intersect_ray(test_origin_query).size() == 0:
					var distance = test_point.distance_to(sample_end)
					if distance < best_distance:
						best_distance = distance
						best_angle = angle * angle_step
			
			if not skip_secondary and best_distance < INF:
				var final_direction = safe_direction.rotated(origin_result.normal, best_angle)
				var final_point = sample_origin + final_direction * sample_origin.distance_to(sample_end)
				
				return _get_safe_point_for_line(
					skeleton,
					sample_origin,
					final_point,
					sample_end,
					true
				)
			else:
				continue
		
		safe_point_found = true
		last_known_safe_point = current_point
		
	if safe_point_found:
		return CollisionData.make(false, true, last_known_safe_point)
	
	# Last resort: return the starting point
	return CollisionData.make(true, false, sample_start)
