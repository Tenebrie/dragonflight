extends Object

class_name IKBone3D

# Parent skeleton
var skeleton: Skeleton3D
# Skeleton bone index
var bone_idx: int
# IK chain may have at most 1 child even if the structure itself has more
var child_bone: IKBone3D
# Parent bone is closer to the root than the current one
var parent_bone_idx: int
# Bone's pivot point
var origin: Vector3
# Bone's target point
var target: Vector3
# Bone's rest pose length
var length: float

static func make(skeleton: Skeleton3D, bone_idx: int, child_idx: int) -> IKBone3D:
	var bone = IKBone3D.new()
	bone.skeleton = skeleton
	bone.bone_idx = bone_idx
	bone.parent_bone_idx = skeleton.get_bone_parent(bone_idx)
	bone.origin = skeleton.get_bone_global_pose(bone_idx).origin
	bone.target = bone._get_target()
	if child_idx != -1:
		bone.length = skeleton.get_bone_global_rest(bone_idx).origin.distance_to(skeleton.get_bone_global_rest(child_idx).origin)
	else:
		bone.length = 1
	return bone

func _get_target() -> Vector3:
	if child_bone:
		return child_bone.origin
	else:
		return origin + skeleton.get_bone_global_pose(bone_idx).basis.y * length

func flush_prepare(data: BoneFlushData) -> BoneFlushData:
	var new_transform = Transform3D(data.transform)
	var parent_transform = Transform3D(data.parent_transform)

	new_transform.origin = origin

	if data.root_roll != 0:
		parent_transform = parent_transform.rotated(parent_transform.basis.y, data.root_roll * PI)

	var forward: Vector3 = (target - origin).normalized()

	var old_up = new_transform.basis.z.rotated(forward, -PI / 2)
	var preferred_up = (old_up - forward * old_up.dot(forward)).normalized()

	var twist = IKUtils.get_roll_difference(new_transform, parent_transform)

	var untwisted_up = preferred_up.rotated(forward, twist)
	preferred_up = preferred_up.lerp(untwisted_up, 1)

	# TODO: Re-add rotational stiffness if needed
	# preferred_up = old_up.lerp(preferred_up, 1 - rotational_stiffness)

	var up = forward.cross(preferred_up).normalized()
	var normal = forward.cross(up).normalized()

	new_transform.basis = Basis(normal, forward, up).orthonormalized()

	return BoneFlushData.from_prep(new_transform, parent_transform)

func flush_changes(data: BoneFlushData) -> void:
	skeleton.set_bone_global_pose(bone_idx, data.transform)

class BoneFlushData:
	var root_roll: float
	var transform: Transform3D
	var parent_transform: Transform3D

	static func from_bone(bone: IKBone3D) -> BoneFlushData:
		var data = BoneFlushData.new()
		data.transform = bone.skeleton.get_bone_global_pose(bone.bone_idx)
		data.parent_transform = bone.skeleton.get_bone_global_pose(bone.parent_bone_idx)
		return data

	static func from_prep(new_transform: Transform3D, new_parent_transform: Transform3D) -> BoneFlushData:
		var data = BoneFlushData.new()
		data.transform = new_transform
		data.parent_transform = new_parent_transform
		return data
