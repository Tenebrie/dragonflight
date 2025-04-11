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