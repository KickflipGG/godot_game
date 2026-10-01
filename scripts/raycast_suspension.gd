class_name RaycastSuspension
extends RayCast3D

## A standalone RayCast3D-based vehicle suspension component for Godot 4.
## Attach this node as a child of a RigidBody3D vehicle node to simulate spring/damper physics.

@export var rest_length: float = 0.5
@export var spring_stiffness: float = 30000.0
@export var damper_stiffness: float = 1500.0
@export var wheel_radius: float = 0.3


func _ready() -> void:
	update_ray_length()


func _physics_process(delta: float) -> void:
	update_ray_length()
	apply_suspension_force(delta)


## Updates the RayCast3D target position according to rest_length and wheel_radius.
func update_ray_length() -> void:
	var total_length: float = rest_length + wheel_radius
	target_position = Vector3(0.0, -total_length, 0.0)


## Calculates and applies suspension spring and damping forces to the parent RigidBody3D.
func apply_suspension_force(delta: float) -> void:
	if delta <= 0.0:
		return

	# Edge case: Raycast is not colliding with any surface
	if not is_colliding():
		return

	# Edge case: Parent node is not a RigidBody3D
	var parent_body: RigidBody3D = get_parent() as RigidBody3D
	if parent_body == null:
		return

	# Direction vectors in world space
	# local -Y is the downward ray direction
	var ray_dir: Vector3 = -global_transform.basis.y.normalized()
	var suspension_up: Vector3 = -ray_dir

	# Distance from ray origin to collision point
	var collision_point: Vector3 = get_collision_point()
	var hit_distance: float = global_position.distance_to(collision_point)

	# Total suspension length ray covers
	var max_ray_length: float = rest_length + wheel_radius
	if hit_distance >= max_ray_length:
		return

	# Calculate spring compression using Hooke's Law (F_spring = k * x)
	# Wheel top contact occurs at hit_distance - wheel_radius from spring origin
	var compression: float = rest_length - (hit_distance - wheel_radius)
	compression = maxf(compression, 0.0)

	var spring_force: float = compression * spring_stiffness

	# Calculate compression velocity of parent body at suspension point
	# point_velocity gets combined linear + angular velocity at collision point
	var point_velocity: Vector3 = parent_body.get_point_velocity(collision_point)

	# Velocity vector projected along ray direction (positive when moving downwards into ground)
	var compression_velocity: float = point_velocity.dot(ray_dir)

	# Damping force (F_damper = c * v)
	var damping_force: float = compression_velocity * damper_stiffness

	# Net force magnitude along suspension up vector:
	# Both spring_force and damping_force push upward along suspension_up during compression
	var total_force_magnitude: float = spring_force + damping_force
	total_force_magnitude = maxf(total_force_magnitude, 0.0)

	var force_vector: Vector3 = suspension_up * total_force_magnitude

	# Apply force to parent RigidBody3D relative to its global center of position
	var force_position: Vector3 = global_position - parent_body.global_position
	parent_body.apply_force(force_vector, force_position)
