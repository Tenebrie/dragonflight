@tool

extends MeshInstance3D
class_name IKBoneTarget3D
## Simple target pointer for the IK system

func _ready() -> void:
	mesh = SphereMesh.new()
	mesh.radius = 0.25
	mesh.height = 0.5

	var sphere_mat = StandardMaterial3D.new()
	sphere_mat.albedo_texture = preload("res://addons/godot-prototype-texture/PNG/grid_512x512/grid_black_512x512.png")
	sphere_mat.uv1_scale = Vector3(4.0, 4.0, 4.0)
	
	mesh.material = sphere_mat
