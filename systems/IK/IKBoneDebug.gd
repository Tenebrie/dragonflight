extends Node3D

class_name IKBoneDebug

var skeleton: Skeleton3D
var debug_mesh: MeshInstance3D

func _ready():
	skeleton = get_parent().get_parent()

	debug_mesh = MeshInstance3D.new()
	debug_mesh.top_level = true
	var mesh = ImmediateMesh.new()
	debug_mesh.mesh = mesh
	var material = StandardMaterial3D.new()
	material.vertex_color_use_as_albedo = true
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	debug_mesh.material_override = material
	add_child(debug_mesh)

func clear_mesh():
	if not debug_mesh:
		return
	debug_mesh.mesh.clear_surfaces()

func print_single_bone(from: Vector3, to: Vector3, color: Color):
	if not debug_mesh:
		return
	debug_mesh.mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	debug_mesh.mesh.surface_set_color(color)
	debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * from)
	debug_mesh.mesh.surface_set_color(color)
	debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * to)
	debug_mesh.mesh.surface_end()

func print_bones(bones: Array[IKBone3D], color: Color):
	if not debug_mesh:
		return
	for bone in bones:
		debug_mesh.mesh.surface_begin(Mesh.PRIMITIVE_LINES)
		debug_mesh.mesh.surface_set_color(color)
		debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * bone.origin)
		debug_mesh.mesh.surface_set_color(color)
		debug_mesh.mesh.surface_add_vertex(skeleton.global_transform * bone.target)
		debug_mesh.mesh.surface_end()