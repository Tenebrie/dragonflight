extends Node3D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if (Input.is_action_pressed("camera_left")):
		rotate_y(deg_to_rad(-135 * delta))
	if (Input.is_action_pressed("camera_right")):
		rotate_y(deg_to_rad(135 * delta))

	pass
