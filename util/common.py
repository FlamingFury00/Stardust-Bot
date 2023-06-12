import math

from util.objects import Vector3, Matrix3

# This file is for small utilities for math and movement


def backsolve(target, car, time, gravity=650):
    # Finds the acceleration required for a car to reach a target in a specific amount of time
    d = target - car.location
    dvx = ((d[0] / time) - car.velocity[0]) / time
    dvy = ((d[1] / time) - car.velocity[1]) / time
    dvz = (((d[2] / time) - car.velocity[2]) / time) + (gravity * time)
    return Vector3(dvx, dvy, dvz)


def cap(x, low, high):
    # caps/clamps a number between a low and high value
    if x < low:
        return low
    elif x > high:
        return high
    return x


def defaultPD(agent, local_target, direction=1.0):
    # points the car towards a given local target.
    # Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    up = agent.me.local(Vector3(0, 0, 1))  # where "up" is in local coordinates
    target_angles = [
        math.atan2(
            local_target[2], local_target[0]
        ),  # angle required to pitch towards target
        math.atan2(
            local_target[1], local_target[0]
        ),  # angle required to yaw towards target
        math.atan2(up[1], up[2]),
    ]  # angle required to roll upright
    # Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    agent.controller.steer = steerPD(target_angles[1], 0) * direction
    agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 6)
    agent.controller.yaw = steerPD(target_angles[1], -agent.me.angular_velocity[2] / 8)
    agent.controller.roll = steerPD(target_angles[2], agent.me.angular_velocity[0] / 3)
    # Returns the angles, which can be useful for other purposes
    return target_angles


def defaultThrottle(agent, target_speed, direction=1.0):
    # accelerates the car to a desired speed using throttle and boost
    car_speed = agent.me.local(agent.me.velocity)[0]
    t = (target_speed * direction) - car_speed
    agent.controller.throttle = cap((t**2) * sign(t) / 1000, -1.0, 1.0)
    agent.controller.boost = (
        True
        if t > 150 and car_speed < 2275 and agent.controller.throttle == 1.0
        else False
    )
    return car_speed


def freestyle_orient(agent, local_target, direction=1.0):
    # points the car towards a given local target.
    # Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    target_angles = [
        math.atan2(
            local_target[2], local_target[0]
        ),  # angle required to pitch towards target
        math.atan2(
            local_target[1], local_target[0]
        ),  # angle required to yaw towards target
        math.atan2(local_target[1], abs(local_target[2])) * sign(local_target[2]),
    ]
    # Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    agent.controller.steer = steerPD(target_angles[1], 0) * direction
    agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 6)
    agent.controller.yaw = steerPD(target_angles[1], -agent.me.angular_velocity[2] / 8)
    agent.controller.roll = steerPD(target_angles[2], agent.me.angular_velocity[0] / 3)
    # Returns the angles, which can be useful for other purposes
    return target_angles


def roll_orient(agent, local_target, direction=1.0):
    # points the car towards a given local target.
    # Direction can be changed to allow the car to steer towards a target while driving backwards
    local_target *= direction
    yaw_angle = math.atan2(local_target[1], local_target[0])
    target_angles = [
        math.atan2(local_target[2], local_target[0])
        * abs(math.cos(yaw_angle)),  # angle required to pitch towards target
        math.atan2(
            local_target[1], local_target[0]
        ),  # angle required to yaw towards target
        math.atan2(local_target[2], abs(local_target[1])) * -sign(local_target[1]),
    ]  # angle required to roll towards target
    # Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    agent.controller.pitch = steerPD(target_angles[0], agent.me.angular_velocity[1] / 6)
    agent.controller.yaw = steerPD(target_angles[1], -agent.me.angular_velocity[2] / 8)
    agent.controller.roll = steerPD(target_angles[2], agent.me.angular_velocity[0] / 3)
    # Returns the angles, which can be useful for other purposes
    return target_angles


def car_ball_collision_offset(car, shot_vector, time_to_jump):
    angle_from_ball = car.forward.angle(shot_vector)
    angle_from_ball_sign = car.forward.anglesign(shot_vector)
    max_angle = get_max_angle(
        cap(time_to_jump - 0.16, 0.001, 1.5), angle_from_ball, 9.11, 0
    )

    pitch_max_angle = get_max_angle(
        cap(time_to_jump, 0.001, 1.5), abs(math.asin(shot_vector.y)) + 0.5, 11, 0.001
    )
    final_pitch = math.asin(car.forward.y) + pitch_max_angle - 0.5

    forward_after_jump = shot_vector.rotate(
        angle_from_ball_sign - max_angle
    ).flatten().normalize() * math.cos(final_pitch) + Vector3(0, 0, 1) * math.sin(
        final_pitch
    )

    predicted_hitbox = car.hitbox
    predicted_hitbox.location = Vector3(0, 0, 0)
    predicted_hitbox.orientation = Matrix3(
        forward_after_jump,
        forward_after_jump.cross(Vector3(0, 0, 1)).normalize(),
        forward_after_jump.cross(
            forward_after_jump.cross(Vector3(0, 0, 1))
        ).normalize(),
    )

    return predicted_hitbox.get_offset(shot_vector)


def predict_car_location(car, time, gravity=650):
    # Finds the cars location after a certain amount of time
    return (
        car.location + car.velocity * time + 0.5 * Vector3(0, 0, -gravity) * time**2
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


def lerp(a, b, t):
    # Linearly interpolate from a to b using t
    # For instance, when t == 0, a is returned, and when t == 1, b is returned
    # Works for both numbers and Vector3s
    return (b - a) * t + a


def in_field(point, radius):
    # determines if a point is inside the standard soccer field
    point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))
    if point[0] > 4080 - radius:
        return False
    elif point[1] > 5060 - radius:
        return False
    elif point[0] > 880 - radius and point[1] > 5105 - radius:
        return False
    elif point[0] > 2650 and point[1] > -point[0] + 8025 - radius:
        return False
    return True


def on_wall(point):
    # determines if a point is on the wall
    point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))
    if point[2] > 400:
        if point[0] > 3600 and 4000 > point[1]:
            return True
        elif 900 < point[0] < 3000 and point[1] > 4900:
            return True
        elif point[0] > 2900 and point[1] > 3800 and 7500 < point[0] + point[1] < 8500:
            return True
        else:
            return False
    else:
        return False


def is_on_wall(point, try_to_reach=False):
    if hasattr(point, "__getitem__"):
        distance = distance_to_wall2(point)[0]

        if try_to_reach:
            return distance < 300 < point[2]
        else:
            return distance < 150 or distance < point[2] + 100 < 400
    else:
        return point.location.z > 50 and not point.airborne


def distance_to_wall(point):
    # determines how close the car is to the wall
    point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))
    if 4096 - point[0] < 5120 - point[1]:
        return 4096 - point[0]
    else:
        return 5120 - point[1]


def distance_to_wall2(point):
    # determines how close the car is to the wall
    abs_point = Vector3(abs(point[0]), abs(point[1]), abs(point[2]))

    distance_to_side_wall = (
        4096 - abs_point[0] if abs_point[1] < 5120 else 800 - abs_point[0]
    )
    distance_to_back_wall = (
        5120 - abs_point[1] if abs_point[0] > 800 else 5920 - abs_point[1]
    )
    distance_to_corner = math.sqrt(2) * ((8064 - abs_point[0] - abs_point[1]) / 2)
    if distance_to_corner > distance_to_side_wall < distance_to_back_wall:
        return distance_to_side_wall, Vector3(sign(point[0]), 0, 0)
    elif distance_to_corner > distance_to_side_wall > distance_to_back_wall:
        return distance_to_back_wall, Vector3(0, sign(point[1]), 0)
    else:
        return distance_to_corner, Vector3(
            sign(point[0]) / math.sqrt(2), sign(point[1]) / math.sqrt(2), 0
        )


def eta(car, ball_location):
    car_to_ball = ball_location - car.location
    # Adding a True to a vector's normalize will have it also return the magnitude of the vector
    direction = car_to_ball.normalize()
    distance = car_to_ball.magnitude()

    # How far the car must turn in order to face the ball, for forward and reverse
    forward_angle = direction.angle(car.orientation[0])
    backward_angle = math.pi - forward_angle

    vel_in_direction = car.velocity.dot(direction)
    # If the car only had to drive in a straight line, we ensure it has enough time to reach the ball (a few assumptions are made)
    forward_time = (distance * 1.05) / cap(
        vel_in_direction + 1000 * car.boost / 30, 1410, 2300
    ) + (forward_angle * 0.318)
    backward_time = (distance * 1.05) / 1400 + (backward_angle * 0.418)

    if forward_time < backward_time or distance > 1500:
        return forward_time, True
    else:
        return backward_time, False


def eta2(car, target=None, direction=None, distance=None):
    if direction != None and distance != None:
        forward_angle = direction.angle(car.forward) * cap(distance - 500, 0, 500) / 500
        car_to_target = target - car.location
        int_vel = cap(car.velocity.magnitude(), 1410, 2300)
        return distance / cap(int_vel + 1000 * car.boost / 30, 1410, 2300) + (
            forward_angle * 0.318
        )
    else:
        car_to_target = target - car.location
        forward_angle = (
            car_to_target.angle(car.forward)
            * cap(car_to_target.magnitude() - 500, 0, 500)
            / 500
        )
        int_vel = cap(car.velocity.magnitude(), 1410, 2300)
        return car_to_target.magnitude() / cap(
            int_vel + 1000 * car.boost / 30, 1410, 2300
        ) + (forward_angle * 0.318)


def find_jump_time(height, double_jump=False):
    int_jump_velocity = 291.667
    gravity_acceleration = -650
    jump_acceleration = 1458.333374
    jump_stop_time = 0.2

    height_after_jump = int_jump_velocity * jump_stop_time + (1 / 2) * (
        gravity_acceleration + jump_acceleration
    ) * math.pow(jump_stop_time, 2)
    double_jump_multiplier = 2 if double_jump else 1

    if height_after_jump < height:
        int_velocity_after_jump = (
            double_jump_multiplier * int_jump_velocity
            + (gravity_acceleration + jump_acceleration) * jump_stop_time
        )
        fin_velocity_after_jump = pom_sqrt(
            math.pow(int_velocity_after_jump, 2)
            + 2 * gravity_acceleration * (height - height_after_jump)
        )
        return (
            jump_stop_time
            + (fin_velocity_after_jump - int_velocity_after_jump) / gravity_acceleration
        )
    else:
        fin_jump_velocity = math.sqrt(
            math.pow(int_jump_velocity, 2)
            + 2 * (gravity_acceleration + jump_acceleration) * height
        )
        return (fin_jump_velocity - int_jump_velocity) / (
            gravity_acceleration + jump_acceleration
        )


def find_shot_angle(s, x, y):
    g = 650  # gravity constant

    if s**4 - g * (g * x**2 + 2 * y * s**2) < 0:
        return (
            math.pi / 6
        )  # not possible to hit target at speed without hitting ground, so angle defaults to 0
    else:
        shot_angle_1 = math.atan2(
            s**2 + math.sqrt(s**4 - g * (g * x**2 + 2 * y * s**2)), g * x
        )
        shot_angle_2 = math.atan2(
            s**2 - math.sqrt(s**4 - g * (g * x**2 + 2 * y * s**2)), g * x
        )
        straight_angle = math.atan2(y, x)

        return (
            shot_angle_1
            if abs(straight_angle - shot_angle_1) < abs(straight_angle - shot_angle_2)
            else shot_angle_2
        )  # returns the smallest angle to hit target


def get_max_angle(t, x, a, vi):
    correction_time = -vi / a
    time_remaining = math.sqrt(4 * x / a)
    peak = a * time_remaining / 2
    above_peak = peak - 5.5
    time_to_peak = peak / a

    if above_peak > 0:
        return (
            a * math.pow(t, 2) / 4
            + cap(correction_time, 0, t) * vi / 2
            - above_peak * cap(t - time_to_peak, 0, 2 * above_peak / a) / 2
        )
    else:
        return a * math.pow(t, 2) / 4 + cap(correction_time, 0, t) * vi / 2


def pom_sqrt(x, both=False):
    if x < 0:
        return -math.sqrt(abs(x))
    else:
        return math.sqrt(x)


def find_rotation(agent, friend):
    my_car_to_ball = agent.ball.location - agent.me.location
    friend_car_to_ball = agent.ball.location - friend.location

    my_distance = my_car_to_ball.magnitude()
    friend_distance = friend_car_to_ball.magnitude()

    goal_to_ball, my_ball_distance = (
        agent.ball.location - agent.friend_goal.location
    ).normalize(True)
    goal_to_me, my_goal_distance = (
        agent.me.location - agent.friend_goal.location
    ).normalize(True)
    goal_to_friend, friend_goal_distance = (
        friend.location - agent.friend_goal.location
    ).normalize(True)

    me_back = my_goal_distance - 200 < my_ball_distance
    friend_back = friend_goal_distance - 200 < my_ball_distance

    my_direction = my_car_to_ball.normalize()
    friend_direction = friend_car_to_ball.normalize()

    left_vector = (agent.foe_goal.left_post - agent.ball.location).normalize()
    right_vector = (agent.foe_goal.right_post - agent.ball.location).normalize()
    my_best_shot_vector = my_direction.clamp(left_vector, right_vector)
    friend_best_shot_vector = friend_direction.clamp(left_vector, right_vector)

    my_angle = my_best_shot_vector.angle(my_car_to_ball)
    friend_angle = friend_best_shot_vector.angle(friend_car_to_ball)

    return (
        (
            not agent.kickoff_flag
            and friend_distance + friend_goal_distance * (friend_angle / math.pi)
            < my_distance + my_goal_distance * (my_angle / math.pi)
            and friend_back == me_back
        )
        or (friend_back and not me_back)
        or (agent.kickoff_flag and friend_distance + 50 < my_distance)
        or (
            agent.kickoff_flag
            and abs(friend_distance - my_distance) < 100
            and sign(agent.me.location.x) == side(agent.team)
        )
    )


def find_slope(shot_vector, car_to_target):
    # Finds the slope of your car's position relative to the shot vector (shot vector is y axis)
    # 10 = you are on the axis and the ball is between you and the direction to shoot in
    # -10 = you are on the wrong side
    # 1.0 = you're about 45 degrees offcenter
    d = shot_vector.dot(car_to_target)
    e = abs(shot_vector.cross((0, 0, 1)).dot(car_to_target))
    return cap(d / e if e != 0 else 10 * sign(d), -3.0, 3.0)


def post_correction(ball_location, left_target, right_target):
    # this function returns target locations that are corrected to account for the ball's radius
    # If the left and right post swap sides, a goal cannot be scored
    ball_radius = 120  # We purposly make this a bit larger so that our shots have a higher chance of success
    goal_line_perp = (right_target - left_target).cross((0, 0, 1))
    left = left_target + (
        (left_target - ball_location).normalize().cross((0, 0, -1)) * ball_radius
    )
    right = right_target + (
        (right_target - ball_location).normalize().cross((0, 0, 1)) * ball_radius
    )
    left = left_target if (left - left_target).dot(goal_line_perp) > 0.0 else left
    right = right_target if (right - right_target).dot(goal_line_perp) > 0.0 else right
    swapped = (
        True
        if (left - ball_location)
        .normalize()
        .cross((0, 0, 1))
        .dot((right - ball_location).normalize())
        > -0.1
        else False
    )
    return left, right, swapped


def quadratic(a, b, c):
    # Returns the two roots of a quadratic
    inside = math.sqrt((b * b) - (4 * a * c))
    if a != 0:
        return (-b + inside) / (2 * a), (-b - inside) / (2 * a)
    else:
        return -1, -1


def shot_valid(agent, shot, threshold=35):
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


def is_closest_kickoff(agent, own_car):
    # distance is calculated as distance + index - boost
    # boost added to allow the bot with the most boost to go for the ball in situations
    # were 2 bots are more or less equally close
    if not len(agent.friends):
        return True
    own_distance = (own_car.location - agent.ball.location).magnitude() + own_car.index
    for car in agent.friends + [agent.me]:
        if own_car.index == car.index:
            continue
        other_distance = (car.location - agent.ball.location).magnitude() + car.index
        if other_distance < own_distance:
            return False
    return True


def is_second_closest_kickof(agent):
    if is_closest_kickoff(agent, agent.me):
        return False
    if len(agent.friends) < 1:
        return False
    own_distance = (
        agent.me.location - agent.ball.location
    ).magnitude() + agent.me.index
    closest_index = -1
    for car in agent.friends:
        if is_closest_kickoff(agent, car):
            closest_index = car.index

    if closest_index == agent.index:
        return False
    for car in agent.friends:
        if car.index == closest_index:
            continue
        other_distance = (car.location - agent.ball.location).magnitude() + car.index
        if other_distance < own_distance:
            return False
    return True


def is_closest(agent, own_car, team_only=False):
    # distance is calculated as distance + index - boost
    # boost added to allow the bot with the most boost to go for the ball in situations
    # were 2 bots are more or less equally close
    if not len(agent.friends) and team_only:
        return True

    ahead_counts = True if team_only and are_no_bots_back(agent) else False

    if is_ahead_of_ball(agent) and not ahead_counts:
        return False
    factor = 1
    if agent.me.location[1] * side(agent.team) < agent.ball.location[1] * side(
        agent.team
    ):
        factor = 5
    if team_only:
        actual_distance_vector = own_car.location - agent.ball.location
        biased_distance_vector = Vector3(
            2 * actual_distance_vector[0],
            factor * actual_distance_vector[1],
            actual_distance_vector[2],
        )
        own_distance = (biased_distance_vector).magnitude() - (10 * own_car.boost)
    else:
        own_distance = (own_car.location - agent.ball.location).magnitude() * factor - (
            10 * own_car.boost
        )
    if not team_only:
        for car in agent.foes:
            factor = 1
            if -car.location[1] * side(agent.team) < -agent.ball.location[1] * side(
                agent.team
            ):
                factor = 5
            other_distance = (
                car.location - agent.ball.location
            ).magnitude() * factor - (10 * car.boost)
            if other_distance < own_distance:
                return False
    for car in agent.friends + [agent.me]:
        if own_car.index == car.index:
            continue
        factor = 1
        if car.location[1] * side(agent.team) < agent.ball.location[1] * side(
            agent.team
        ):
            factor = 5

        if team_only:
            other_actual_distance_vector = car.location - agent.ball.location
            other_biased_distance_vector = Vector3(
                2 * other_actual_distance_vector[0],
                factor * other_actual_distance_vector[1],
                other_actual_distance_vector[2],
            )
            other_distance = (other_biased_distance_vector).magnitude() - (
                10 * car.boost
            )
        else:
            other_distance = (
                car.location - agent.ball.location
            ).magnitude() * factor - (10 * car.boost)
        if is_ahead_of_ball_2(agent, car.location, agent.team) and not ahead_counts:
            continue
        if other_distance < own_distance:
            return False

    return True


def is_second_closest(agent):
    if is_closest(agent, agent.me, True):
        return False
    if is_ahead_of_ball(agent):
        return False
    if len(agent.friends) < 1:
        return False
    factor = 1
    if agent.me.location[1] * side(agent.team) < agent.ball.location[1] * side(
        agent.team
    ):
        factor = 5

    actual_distance_vector = agent.me.location - agent.ball.location
    biased_distance_vector = Vector3(
        2 * actual_distance_vector[0],
        factor * actual_distance_vector[1],
        actual_distance_vector[2],
    )
    own_distance = (biased_distance_vector).magnitude() - (10 * agent.me.boost)
    closest_index = -1
    for car in agent.friends:
        if is_closest(agent, car, True):
            closest_index = car.index

    if closest_index == agent.index:
        return False
    for car in agent.friends:
        if car.index == closest_index:
            continue
        factor = 1
        if car.location[1] * side(agent.team) < agent.ball.location[1] * side(
            agent.team
        ):
            factor = 5

        other_actual_distance_vector = car.location - agent.ball.location
        other_biased_distance_vector = Vector3(
            2 * other_actual_distance_vector[0],
            factor * other_actual_distance_vector[1],
            other_actual_distance_vector[2],
        )
        other_distance = (other_biased_distance_vector).magnitude() - (10 * car.boost)
        if other_distance < own_distance:
            return False
    return True


def is_ball_going_towards_goal(agent):
    if not agent.ball.velocity[1]:
        return False
    if agent.ball.velocity[1] * side(agent.team) < 0:
        return False
    ball_position_x = agent.ball.location[0] * side(agent.team)
    ball_position_y = agent.ball.location[1] * side(agent.team)
    ball_velocity_x = agent.ball.velocity[0] * side(agent.team)
    ball_velocity_y = agent.ball.velocity[1] * side(agent.team)
    m = ball_velocity_x / ball_velocity_y
    y = 5120 - ball_position_y
    x = m * y + ball_position_x
    return x >= -896 and x <= 893


def is_ball_on_back_wall(agent, enemy=True):
    y = side(agent.team) * -4950
    if not enemy:
        y *= -1
    possible = False
    if agent.team == 0:
        if (agent.ball.location[1] > y and enemy) or (
            agent.ball.location[1] < y and not enemy
        ):
            possible = True

    if agent.team == 1:
        if (agent.ball.location[1] < y and enemy) or (
            agent.ball.location[1] > y and not enemy
        ):
            possible = True

    if possible and abs(agent.ball.location[0]) < 1000:
        if agent.ball.location[2] > 1100:
            return True
        return False
    return False


def is_ahead_of_ball(agent):
    return (
        agent.me.location[1] > agent.ball.location[1] + 500 and agent.team == 0
    ) or (agent.me.location[1] < agent.ball.location[1] - 500 and agent.team == 1)


def is_ahead_of_ball_2(agent, location, team):
    return (location[1] > agent.ball.location[1] + 500 and team == 0) or (
        location[1] < agent.ball.location[1] - 500 and team == 1
    )


def get_desired_zone(agent):
    ball_zone = get_location_zone(agent.ball.location)
    possible_zones = []
    if agent.team == 0:
        if ball_zone >= 7:
            for i in range(7, 10):
                possible_zones.append(i)
        elif ball_zone >= 4:
            for i in range(7, 10):
                if ball_zone == 4 and i == 9:
                    continue
                if ball_zone == 6 and i == 7:
                    continue
                possible_zones.append(i)
        else:
            for i in range(4, 7):
                if ball_zone == 1 and i == 6:
                    continue
                if ball_zone == 3 and i == 4:
                    continue
                possible_zones.append(i)

    if agent.team == 1:
        if ball_zone <= 3:
            for i in range(1, 4):
                possible_zones.append(i)
        elif ball_zone <= 6:
            for i in range(1, 4):
                if ball_zone == 4 and i == 3:
                    continue
                if ball_zone == 6 and i == 1:
                    continue
                possible_zones.append(i)
        else:
            for i in range(4, 7):
                if ball_zone == 7 and i == 6:
                    continue
                if ball_zone == 9 and i == 4:
                    continue
                possible_zones.append(i)
    possible_zones = reduce_possible_zones(agent, possible_zones)
    if not len(possible_zones):
        return -1
    return get_closest_zone(agent, possible_zones)


def reduce_possible_zones(agent, possible_zones):
    goal = 2 if agent.team == 1 else 8
    for car in agent.friends:
        zone = get_location_zone(car.location)
        zone_5, center = zone_5_positioning(agent)
        if zone in possible_zones:
            if zone == goal:
                if is_ball_centering(agent, True) or (
                    get_location_zone(agent.ball.location) == 5
                ):
                    if friendly_cars_in_front_of_goal(agent) < 2:
                        continue
                if (car.location - agent.friend_goal.location).magnitude() > (
                    agent.me.location - agent.friend_goal.location
                ).magnitude():
                    continue
                else:
                    possible_zones.remove(zone)
            elif zone == 5 and center:
                if (car.location - zone_5).magnitude() < (
                    agent.me.location - zone_5
                ).magnitude():
                    possible_zones.remove(zone)
            elif (car.location - zone_center(zone)).magnitude() < (
                agent.me.location - zone_center(zone)
            ).magnitude():
                possible_zones.remove(zone)
    return possible_zones


def friendly_cars_in_front_of_goal(agent):
    count = 0
    goal = 2 if agent.team == 1 else 8
    for car in agent.friends:
        if get_location_zone(car.location) == goal:
            count += 1
    return count


def get_location_zone(vector):
    i = 1
    j = 2
    y = vector[1]
    x = vector[0]
    if y >= 1707:
        i = 0
    elif y <= -1707:
        i = 2
    if x >= 1356:
        j = 1
    if x <= -1356:
        j = 3
    return i * 3 + j


def get_closest_zone(agent, zones):
    closest = -1
    closest_distance = 9999999
    if 2 in zones:
        return 2
    if 5 in zones:
        return 5
    if 8 in zones:
        return 8
    for i in zones:
        if i == 10:
            print("HELP ZONE 10")
        x = 0
        y = 0
        if i < 4:
            y = 2 * 1707
        if i > 6:
            y = -2 * 1707
        if i == 1 or i == 4 or i == 7:
            x = 2 * 1356
        if i == 3 or i == 6 or i == 9:
            x = -2 * 1356
        distance = (agent.me.location - Vector3(x, y, 0)).magnitude()
        if distance < closest_distance:
            closest_distance = distance
            closest = i
    return closest


def zone_center(zone):
    if zone < 1 or zone > 9:
        return Vector3(0, 0, 0)
    x = 0
    y = 0
    z = 0
    if zone == 2 or zone == 8:
        return Vector3(0, -5000 if zone == 8 else 5000, 0)
    if zone < 4:
        y = 4520.0
    if zone > 6:
        y = -4520.0
    if zone == 1 or zone == 4 or zone == 7:
        x = 2.0 * 1356.0
    if zone == 3 or zone == 6 or zone == 9:
        x = -2.0 * 1356.0
    return Vector3(x, y, z)


def zone_5_positioning(agent):
    center = is_ball_centering(agent)
    ball_y = agent.ball.location[1]
    d_y = ball_y + 1707 * side(agent.team)
    return Vector3(0, 2 * d_y / 3, 0), center


def are_no_bots_back(agent):
    if not is_ahead_of_ball(agent) or is_closest(agent, agent.me):
        return False
    for car in agent.friends:
        if not is_ahead_of_ball_2(agent, car.location, agent.team):
            return False
    return True


def is_ball_centering(agent, friendly=False):
    struct = agent.get_ball_prediction_struct()
    for i in range(15, struct.num_slices, 10):
        ball_location = Vector3(struct.slices[i].physics.location)
        if abs(ball_location[0]) < 1500:
            if ball_location[1] * side(agent.team) < -3000 and not friendly:
                if ball_location[2] < 1500:
                    return True
            elif ball_location[1] * side(agent.team) > 3000 and friendly:
                if ball_location[2] < 1500:
                    return True
    return False


def opponent_car_by_index(agent, index):
    for car in agent.foes:
        if car.index == index:
            return car


def demo_rotation(agent):
    possible_cars = []
    for car in agent.foes:
        if car.location[1] * side(agent.team) < -4000:
            distance_to_target = (agent.me.location - car.location).magnitude()
            velocity = (agent.me.velocity).magnitude()
            velocity_needed = 2200 - velocity
            time_boosting_required = velocity_needed / 991.666
            boost_required = 33.3 * time_boosting_required
            distance_required = velocity * time_boosting_required + 0.5 * 991.666 * (
                time_boosting_required**2
            )
            if velocity < 2200:
                if agent.me.boost < boost_required:
                    continue
                elif distance_required > distance_to_target:
                    continue
                possible_cars.append(car)
            else:
                possible_cars.append(car)
    if not len(possible_cars):
        return False, -1
    if len(possible_cars) == 1:
        return True, possible_cars[0].index
    else:
        possible_cars.sort(
            key=lambda car: (agent.foe_goal.location - car.location).magnitude()
        )
        return True, possible_cars[0].index


def friends_ahead_of_ball(agent):
    count = 0
    for car in agent.friends:
        if (car.location[1] > agent.ball.location[1] + 500 and agent.team == 0) or (
            car.location[1] < agent.ball.location[1] - 500 and agent.team == 1
        ):
            count += 1
    return count


def friends_attacking(agent):
    count = 0
    for car in agent.friends:
        if car.location.y * side(agent.team) > 0:
            count += 1
    return count


def is_last_one_back(agent):
    # don't use on defence
    my_y = agent.me.location[1] * side(agent.team)
    for car in agent.friends:
        if car.location[1] * side(agent.team) > my_y:
            return False
    return True


def in_goal_area(agent):
    if abs(agent.me.location[1]) > 5050:
        if abs(agent.me.location[0]) < 880:
            return True
    return False


def detect_demo(agent):
    for car in agent.foes:
        if not car.airborne:
            distance_to_target = (agent.me.location - car.location).magnitude()
            velocity = (car.velocity).magnitude()
            velocity_needed = 2200 - velocity
            time_boosting_required = velocity_needed / 991.666
            boost_required = 33.3 * time_boosting_required
            distance_required = velocity * time_boosting_required + 0.5 * 991.666 * (
                time_boosting_required**2
            )
            time_to_target = distance_to_target / velocity
            aim_point = car.location + time_to_target * velocity
            my_future_location = (
                agent.me.location + time_to_target * agent.me.velocity.magnitude()
            )
            can_demo = car.supersonic or (
                distance_required < distance_to_target and boost_required < car.boost
            )
            if (aim_point - my_future_location).magnitude() and can_demo:
                if time_to_target < 0.75:
                    return False, car  # disabled
    return False, None
