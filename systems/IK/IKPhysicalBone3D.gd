extends PhysicalBone3D

class_name IKPhysicalBone3D

var ik_target: Vector3 = Vector3.ZERO

func set_ik_target(target: Vector3):
	self.ik_target = target

var rest_forward: Vector3

func set_rest_forward(forward: Vector3):
	self.rest_forward = forward

func _integrate_forces(state: PhysicsDirectBodyState3D):
	state.apply_central_force(Vector3.DOWN * 98)
	state.apply_central_force(rest_forward * 300)
	# state.linear_velocity *= 0.5
	# state.angular_velocity *= 0.5
	# state.total_linear_damp = 1
	# state.total_angular_damp = 1
	# var temp = global_position + joint_offset.origin * basis.inverse()
	# var actual_from = temp * skeleton.global_transform
	# state.linear_velocity = Vector3.ZERO
	# state.angular_velocity = Vector3.ZERO
	# var max_force = 1000
	# var joint_pos = global_position + joint_offset.origin * basis.inverse()

	# var diff = ik_target - joint_pos
	# var magnitude = diff.length() * 100
	# var direction = diff.normalized()
	# print(direction * min(magnitude, max_force))
	# state.apply_force(direction * min(magnitude, max_force), -joint_offset.origin * basis.inverse())
	# # state.linear_velocity = Vector3.DOWN
	# # state.angular_velocity = Vector3.ZERO
	
	pass
