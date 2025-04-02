extends Node3D

var skeleton: Skeleton3D

var poses = {
	"rest": {
		"Neck.001": {
			"rot": Vector3(-90, 0, 0),
		},
		"Neck.002": {
			"rot": Vector3(45, 0, 0),
		},
		"Head": {
			"rot": Vector3(45, 0, 0),
		},
		"Wing.001": {
			"rot": Vector3(-1, -23.7, -155.3),
		},
		"Wing.002": {
			"rot": Vector3(14.7, 46, -167.2),
		},
		"Wing.Finger.001": {
			"rot": Vector3(11, 4.9, -177.8),
		},
		"Wing.Finger.002": {
			"rot": Vector3(10.2, 2.8, 164.9),
		},
		"Wing.Finger.003": {
			"rot": Vector3(8.6, 3.7, 160.6),
		},
		"Wing.Finger.004": {
			"rot": Vector3(6.3, 12.3, 150.6),
		},
		"Wing.Finger.005": {
			"rot": Vector3(-28, 10.5, -40.4),
		},
	},
	"glide": {
		"Neck.001": {
			"rot": Vector3(-90, 0, 0),
		},
		"Neck.002": {
			"rot": Vector3(45, 0, 0),
		},
		"Head": {
			"rot": Vector3(45, 0, 0),
		},
		"Wing.001": {
			"rot": Vector3(0, 0, -90),
			"min": Vector3(-180, -50, -180),
			"max": Vector3(180, 60, 180),
		},
		"Wing.002": {
			"rot": Vector3(0, 0, 45),
		},
		"Wing.Finger.001": {
			"rot": Vector3(0, 0, 0),
		},
		"Wing.Finger.002": {
			"rot": Vector3(0, 0, -45),
		},
		"Wing.Finger.003": {
			"rot": Vector3(0, 0, -75),
		},
		"Wing.Finger.004": {
			"rot": Vector3(0, 0, -90),
		},
		"Wing.Finger.005": {
			"rot": Vector3(0, 0, 0),
		},
	},
}

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	skeleton = $Armature/Skeleton3D
	return
	
	# Introduce some noise into poses
	for pose in poses.keys():
		for bone_name in poses[pose].keys():
			var rot = poses[pose][bone_name]["rot"]
			rot.x += randf_range(-0.5, 0.5)
			rot.y += randf_range(-0.5, 0.5)
			rot.z += randf_range(-0.5, 0.5)
			poses[pose][bone_name]["rot"] = rot

func get_bone_rotation(bone_name: String) -> Vector3:
	var bone_index = skeleton.find_bone(bone_name)
	if bone_index != -1:
		var in_radians = skeleton.get_bone_pose_rotation(bone_index).get_euler()
		var in_degrees = Vector3(
			rad_to_deg(in_radians.x),
			rad_to_deg(in_radians.y),
			rad_to_deg(in_radians.z)
		)
		return in_degrees
	else:
		print("Bone not found: ", bone_name)
		return Vector3.ZERO

func set_bone_rotation(bone_name: String, bone_rot: Vector3) -> void:
	var bone_index = skeleton.find_bone(bone_name)
	if bone_index != -1:
		var fixed_rot = Vector3(
			bone_rot.x if bone_rot.x != 0 else 0.1,
			bone_rot.y if bone_rot.y != 0 else 0.1,
			bone_rot.z if bone_rot.z != 0 else 0.1
		)

		var quart = Quaternion.from_euler(fixed_rot)
		quart = apply_procedurals_quart(state, bone_name, quart)
	
		skeleton.set_bone_pose_rotation(bone_index, quart)
	else:
		print("Bone not found: ", bone_name)

var flap_state: float = -3
var facing_x: float = 0
var facing_y: float = 0

var state = "rest"

var state_weights = {
	"rest": 0.0,
	"glide": 0.0,
}

# When mouse moves, rotate the dragon
func _input(event):
	if event is InputEventMouseMotion:
		var mouse_pos = event.position
		var viewport = get_viewport().get_visible_rect().size
		var mouse_x = mouse_pos.x / viewport.x
		facing_x = mouse_x
		var mouse_y = mouse_pos.y / viewport.y
		facing_y = mouse_y

func lerp_angle_degrees(a: float, b: float, t: float) -> float:
	var delta = fposmod(b - a, 360)
	if delta > 180:
		delta -= 360
	return a + delta * t

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	var wing_target: Node3D = $Armature/Skeleton3D/LeftWingBaseTarget
	if wing_target != null:
		wing_target.translate(Vector3(-3, 0, 0) * delta)
		return

	return
	#Increment flap_state by delta
	# flap_state += delta / 10
	# print(flap_state)
	var controlled_bones = [
		"Head",
		"Neck.001",
		"Neck.002",
		"Wing.001",
		"Wing.002",
		"Wing.Finger.001",
		"Wing.Finger.002",
		"Wing.Finger.003",
		"Wing.Finger.004",
		"Wing.Finger.005",
	]

	if state == "rest":
		state_weights["rest"] = lerp(state_weights["rest"], 1.0, delta * 5)
		state_weights["glide"] = lerp(state_weights["glide"], 0.0, delta * 5)
	elif state == "glide":
		state_weights["glide"] = lerp(state_weights["glide"], 1.0, delta * 5)
		state_weights["rest"] = lerp(state_weights["rest"], 0.0, delta * 5)

	if Input.is_action_pressed("wing_flap"):
		state = "glide"
		flap_state += delta * 5
		if flap_state > 1.0:
			flap_state = 1.0
		elif flap_state < 0.0:
			flap_state = 0.0
	elif state == "glide":
		flap_state -= delta * 10
		state_weights["glide"] -= delta * 3
		if flap_state < -4.0:
			state = "rest"
	else:
		flap_state = lerp(flap_state, 0.0, delta * 10)

	for bone_name in controlled_bones:
		var bone_rot
		if bone_name == "Head" or bone_name == "Neck.001" or bone_name == "Neck.002":
			bone_rot = get_bone_rotation(bone_name)
		else:
			bone_rot = get_bone_rotation(bone_name + ".L")
		# var desired_rot = poses[state][bone_name]["rot"]

		var desired_rot_rest = poses["rest"][bone_name]["rot"]
		var desired_rot_glide = poses["glide"][bone_name]["rot"]
		var desired_rot = Vector3(
			lerp_angle_degrees(desired_rot_rest.x, desired_rot_glide.x, state_weights["glide"]),
			lerp_angle_degrees(desired_rot_rest.y, desired_rot_glide.y, state_weights["glide"]),
			lerp_angle_degrees(desired_rot_rest.z, desired_rot_glide.z, state_weights["glide"])
		)

		desired_rot = apply_procedurals(state, bone_name, desired_rot)

		var lerp_duration = 200 # milliseconds
		bone_rot.x = lerp_angle(deg_to_rad(bone_rot.x), deg_to_rad(desired_rot.x), (1 / (lerp_duration / 1000.0)) * delta)
		bone_rot.y = lerp_angle(deg_to_rad(bone_rot.y), deg_to_rad(desired_rot.y), (1 / (lerp_duration / 1000.0)) * delta)
		bone_rot.z = lerp_angle(deg_to_rad(bone_rot.z), deg_to_rad(desired_rot.z), (1 / (lerp_duration / 1000.0)) * delta)

		if poses["glide"][bone_name].has("min") and poses["glide"][bone_name].has("max"):
			var min_value = poses["glide"][bone_name]["min"]
			var max_value = poses["glide"][bone_name]["max"]
			bone_rot.x = clamp(bone_rot.x, deg_to_rad(min_value.x), deg_to_rad(max_value.x))
			bone_rot.y = clamp(bone_rot.y, deg_to_rad(min_value.y), deg_to_rad(max_value.y))
			bone_rot.z = clamp(bone_rot.z, deg_to_rad(min_value.z), deg_to_rad(max_value.z))

		if bone_name == "Head" or bone_name == "Neck.001" or bone_name == "Neck.002":
			set_bone_rotation(bone_name, bone_rot)
		else:
			set_bone_rotation(bone_name + ".L", bone_rot)
			set_bone_rotation(bone_name + ".R", -bone_rot)

func apply_procedurals(state: String, bone_name: String, rot_in_degs: Vector3):
	if (bone_name == "Wing.001"):
		rot_in_degs.y += max(-1, flap_state) * 60
	if (bone_name == "Head"):
		rot_in_degs.x += (facing_y - 0.5) * 45
		rot_in_degs.z += (facing_x - 0.5) * 90
	if (bone_name == "Neck.001"):
		rot_in_degs.x += (facing_y - 0.5) * 60
	if (bone_name == "Neck.002"):
		rot_in_degs.x += (facing_y - 0.5) * 30
		rot_in_degs.z += (facing_x - 0.5) * 90
	return rot_in_degs

func apply_procedurals_quart(state: String, bone_name: String, quart: Quaternion):
	if (bone_name == "Neck.001"):
		var rot = (facing_x - 0.5) * 0.025
		quart.y -= rot
		quart.z += rot

	return quart.normalized()
