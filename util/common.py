import math
from util.objects import Vector3

# This file is for small utilities for math and movement


def pid_controller(target_angle, current_angle, previous_error, dt, kP, kI, kD):
    error = target_angle - current_angle
    integral = previous_error + error * dt
    derivative = (error - previous_error) / dt
    output = kP * error + kI * integral + kD * derivative
    return output, error


def eta(car, target=None, direction=None, distance=None):
    if direction is not None and distance is not None:
        forward_angle = direction.angle(car.forward) * cap(distance - 500, 0, 500) / 500
        car_to_target = target - car.location
    else:
        car_to_target = target - car.location
        forward_angle = (
            car_to_target.angle(car.forward)
            * cap(car_to_target.magnitude() - 500, 0, 500)
            / 500
        )

    acceleration = 1600  # Acceleration in uu/s^2
    drag_coefficient = 0.05  # Drag coefficient
    int_vel = cap(car.velocity.magnitude(), 1410, 2300)

    # Calculate the time it takes to reach the target considering acceleration and drag
    time_to_reach_target = (
        -int_vel
        + math.sqrt(int_vel**2 + 2 * acceleration * car_to_target.magnitude())
    ) / (acceleration - drag_coefficient * int_vel)

    return time_to_reach_target + (forward_angle * 0.318)


def distance_to_wall(point):
    abs_point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))

    distance_to_side_wall = (
        4096 - abs_point[0] if abs_point[1] < 5120 else 800 - abs_point[0]
    )
    distance_to_back_wall = (
        5120 - abs_point[1] if abs_point[0] > 800 else 5920 - abs_point[1]
    )
    distance_to_corner = math.sqrt(2) * ((8064 - abs_point[0] - abs_point[1]) / 2)

    min_distance = min(distance_to_side_wall, distance_to_back_wall, distance_to_corner)
    if min_distance == distance_to_side_wall:
        return distance_to_side_wall, Vector3(sign(point[0]), 0, 0)
    elif min_distance == distance_to_back_wall:
        return distance_to_back_wall, Vector3(0, sign(point[1]), 0)
    else:
        return distance_to_corner, Vector3(
            sign(point[0]) / math.sqrt(2), sign(point[1]) / math.sqrt(2), 0
        )


def is_on_wall(point, try_to_reach=False):
    if hasattr(point, "__getitem__"):
        distance = distance_to_wall(point)[0]

        if try_to_reach:
            return distance < 300 < point[2]
        else:
            return distance < 150 or distance < point[2] + 100 < 400
    else:
        return point.location.z > 50 and not point.airborne


def within_turn_radius(car, location):
    location = location.flatten()
    turn_radius = find_turn_radius(
        math.cos(car.velocity.angle(car.forward)) * car.velocity.magnitude()
    )

    left_turn_center = (
        car.location + car.left.flatten().normalize() * turn_radius
    ).flatten()
    right_turn_center = (
        car.location - car.left.flatten().normalize() * turn_radius
    ).flatten()

    return (
        location.distance(left_turn_center) < turn_radius
        or location.distance(right_turn_center) < turn_radius
    )


def find_turn_radius(speed):
    if speed <= 500:
        radius = lerp(145, 251, speed / 500)
    elif speed <= 1000:
        radius = lerp(251, 425, (speed - 500) / 500)
    elif speed <= 1500:
        radius = lerp(425, 727, (speed - 1000) / 500)
    elif speed <= 1750:
        radius = lerp(727, 909, (speed - 1500) / 250)
    else:
        radius = lerp(909, 1136, (speed - 1750) / 550)

    return radius


def backsolve(target, car, time, gravity=650):
    # Finds the acceleration required for a car to reach a target in a specific amount of time
    if time <= 0.001:
        return Vector3(0, 0, 0)

    velocity_required = (target - car.location) / time
    acceleration_required = velocity_required - car.velocity
    acceleration_required[2] += gravity * time
    return acceleration_required


def cap(x, low, high):
    # caps/clamps a number between a low and high value
    if x < low:
        return low
    elif x > high:
        return high
    return x


def defaultPD(agent, local_target, direction=1.0, dt=0.016):
    # points the car towards a given local target.
    # Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    up = agent.me.local(Vector3(0, 0, 1))  # where "up" is in local coordinates
    target_angles = [
        # angle required to pitch towards target
        math.atan2(local_target[2], local_target[0]),
        # angle required to yaw towards target
        math.atan2(local_target[1], local_target[0]),
        math.atan2(up[1], up[2]),
    ]  # angle required to roll upright
    # Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs

    kP = [1.0, 1.0, 1.0]  # Valori di guadagno proporzionale
    kI = [0.1, 0.1, 0.1]  # Valori di guadagno integrale
    kD = [0.2, 0.2, 0.2]  # Valori di guadagno derivativo

    for i in range(3):
        output, error = pid_controller(
            target_angles[i], 0, agent.previous_errors[i], dt, kP[i], kI[i], kD[i]
        )
        agent.previous_errors[i] = error

        output = cap(output, -1.0, 1.0)

        if i == 0:
            agent.controller.pitch = output
        elif i == 1:
            agent.controller.yaw = output
        else:
            agent.controller.roll = output

    agent.controller.steer = steerPD(target_angles[1], 0) * direction
    # agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 4)
    # agent.controller.yaw = steerPD(target_angles[1], -agent.me.angular_velocity[2] / 4)
    # agent.controller.roll = steerPD(target_angles[2], agent.me.angular_velocity[0] / 2)
    # Returns the angles, which can be useful for other purposes
    return target_angles


def defaultThrottle(agent, target_speed, direction=1.0, dt=0.016):
    car_speed = agent.me.local(agent.me.velocity)[0]
    kP = 1.0
    kI = 0.1
    kD = 0.2
    throttle_output, throttle_error = pid_controller(
        target_speed * direction,
        car_speed,
        agent.previous_throttle_error,
        dt,
        kP,
        kI,
        kD,
    )
    t = (target_speed * direction) - car_speed

    agent.previous_throttle_error = throttle_error
    agent.controller.throttle = cap(throttle_output, -1.0, 1.0)
    agent.controller.boost = (
        True
        if t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0
        else False
    )
    return car_speed


def in_field(point, radius):
    # determines if a point is inside the standard soccer field
    point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))
    if point[0] > 4080 - radius:
        return False
    elif point[1] > 5900 - radius:
        return False
    elif point[0] > 880 - radius and point[1] > 5105 - radius:
        return False
    elif point[0] > 2650 and point[1] > -point[0] + 8025 - radius:
        return False
    return True


def find_slope(shot_vector, car_to_target):
    # Finds the slope of your car's position relative to the shot vector (shot vector is y axis)
    # 10 = you are on the axis and the ball is between you and the direction to shoot in
    # -10 = you are on the wrong side
    # 1.0 = you're about 45 degrees offcenter
    d = shot_vector.dot(car_to_target)
    e = abs(shot_vector.cross((0, 0, 1)).dot(car_to_target))
    return cap(d / e if e != 0 else 10 * sign(d), -3.0, 3.0)


def post_correction(ball_location, left_target: Vector3, right_target: Vector3):
    # this function returns target locations that are corrected to account for the ball's radius
    # it also checks to make sure the ball can fit between the corrected locations
    # We purposely make this a bit larger so that our shots have a higher chance of success
    ball_radius = 110
    goal_line_perp = (right_target - left_target).cross((0, 0, 1))
    left_adjusted = left_target + (
        (left_target - ball_location).normalize().cross((0, 0, -1)) * ball_radius
    )
    right_adjusted = right_target + (
        (right_target - ball_location).normalize().cross((0, 0, 1)) * ball_radius
    )
    left_corrected = (
        left_target
        if (left_adjusted - left_target).dot(goal_line_perp) > 0.0
        else left_adjusted
    )
    right_corrected = (
        right_target
        if (right_adjusted - right_target).dot(goal_line_perp) > 0.0
        else right_adjusted
    )

    difference = right_corrected - left_corrected
    new_goal_line = difference.normalize()
    new_goal_width = difference.magnitude()
    new_goal_perp = new_goal_line.cross((0, 0, 1))
    goal_center = left_corrected + (new_goal_line * new_goal_width * 0.5)
    ball_to_goal = (goal_center - ball_location).normalize()

    ball_fits = new_goal_width * abs(new_goal_perp.dot(ball_to_goal)) > ball_radius * 2
    return left_corrected, right_corrected, ball_fits


def quadratic(a, b, c):
    discriminant = (b * b) - (4 * a * c)
    if discriminant < 0:
        return None, None
    inside = math.sqrt(discriminant)
    if a != 0:
        return (-b + inside) / (2 * a), (-b - inside) / (2 * a)
    else:
        return -1, -1


def shot_valid(agent, shot, threshold=60):
    # Returns True if the ball is still where the shot anticipates it to be
    # First finds the two closest slices in the ball prediction to shot's intercept_time
    # threshold controls the tolerance we allow the ball to be off by
    slices = agent.get_ball_prediction_struct().slices
    soonest = 0
    latest = len(slices) - 1
    while len(slices[soonest : latest + 1]) > 2:
        midpoint = (soonest + latest) // 2
        if slices[midpoint].game_seconds > shot.intercept_time:
            latest = midpoint
        else:
            soonest = midpoint
    # preparing to interpolate between the selected slices
    dt = slices[latest].game_seconds - slices[soonest].game_seconds
    time_from_soonest = shot.intercept_time - slices[soonest].game_seconds
    slopes = (
        Vector3(slices[latest].physics.location)
        - Vector3(slices[soonest].physics.location)
    ) * (1 / dt)
    # Determining exactly where the ball will be at the given shot's intercept_time
    predicted_ball_location = Vector3(slices[soonest].physics.location) + (
        slopes * time_from_soonest
    )
    # Comparing predicted location with where the shot expects the ball to be
    return (shot.ball_location - predicted_ball_location).magnitude() < threshold


def side(x):
    # returns -1 for blue team and 1 for orange team
    if x == 0:
        return -1
    return 1


def sign(x):
    # returns the sign of a number, -1, 0, +1
    if x < 0.0:
        return -1
    elif x > 0.0:
        return 1
    else:
        return 0.0


def steerPD(angle, rate):
    # A Proportional-Derivative control loop used for defaultPD
    return cap(((35 * (angle + rate)) ** 3) / 10, -1.0, 1.0)


def lerp(a, b, t):
    # Linearly interpolate from a to b using t
    # For instance, when t == 0, a is returned, and when t == 1, b is returned
    # Works for both numbers and Vector3s
    return (b - a) * t + a


def invlerp(a, b, v):
    # Inverse linear interpolation from a to b with value v
    # For instance, it returns 0 if v == a, and returns 1 if v == b, and returns 0.5 if v is exactly between a and b
    # Works for both numbers and Vector3s
    return (v - a) / (b - a)
