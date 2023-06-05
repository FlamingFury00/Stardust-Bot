from util.routines import *


def find_hits(agent, targets):
    hits = {name: [] for name in targets}
    struct = agent.get_ball_prediction_struct()

    for i in range(struct.num_slices):
        intercept_time = struct.slices[i].game_seconds
        time_remaining = intercept_time - agent.time
        if time_remaining > 0:
            ball_location = Vector3(struct.slices[i].physics.location)
            ball_velocity = Vector3(struct.slices[i].physics.velocity).magnitude()

            if abs(ball_location[1]) > 5250:
                break

            car_to_ball = ball_location - agent.me.location
            direction = car_to_ball.normalize()
            distance = car_to_ball.magnitude()

            forward_angle = direction.angle(agent.me.forward)
            backward_angle = math.pi - forward_angle

            forward_time = time_remaining - (forward_angle * 0.318)
            backward_time = time_remaining - (backward_angle * 0.418)

            forward_speed = distance * 1.025 / forward_time
            backward_speed = distance * 1.05 / backward_time

            forward_boost = agent.me.boost > distance / 100
            forward_velocity = max(1400, 0.8 * agent.me.velocity.flatten().magnitude())

            forward_flag = forward_time > 0.0 and forward_speed < (
                2299 if forward_boost else forward_velocity
            )
            backward_flag = (
                distance < 1500 and backward_time > 0.0 and backward_speed < 1200
            )

            if forward_flag or backward_flag:
                for pair in targets:
                    left, right, fits = post_correction(
                        ball_location, targets[pair][0], targets[pair][1]
                    )
                    if fits:
                        left_vector = (left - ball_location).normalize()
                        right_vector = (right - ball_location).normalize()
                        best_shot_vector = direction.clamp(left_vector, right_vector)

                        if in_field(ball_location - (200 * best_shot_vector), 1):
                            slope = find_slope(
                                best_shot_vector.flatten(), car_to_ball.flatten()
                            )
                            if forward_flag:
                                if (
                                    ball_location[2] <= 300
                                    or (
                                        not in_field(ball_location, 200)
                                        and not in_field(agent.me.location, 100)
                                    )
                                ) and slope > 0.0:
                                    hits[pair].append(
                                        jump_shot(
                                            ball_location,
                                            intercept_time,
                                            best_shot_vector,
                                            slope,
                                        )
                                    )
                                if (
                                    ball_location[2] > 325
                                    and slope > 1
                                    and cap(ball_location[2] - 400, 100, 2000) * 0.1
                                    < agent.me.boost
                                ):
                                    if (
                                        abs(
                                            (car_to_ball / forward_time)
                                            - agent.me.velocity
                                        ).magnitude()
                                        - 300
                                        < 400 * forward_time
                                    ):
                                        hits[pair].append(
                                            aerial_shot(
                                                ball_location,
                                                intercept_time,
                                                best_shot_vector,
                                                slope,
                                            )
                                        )
                            elif (
                                backward_flag
                                and ball_location[2] <= 280
                                and slope > 0.25
                            ):
                                hits[pair].append(
                                    jump_shot(
                                        ball_location,
                                        intercept_time,
                                        best_shot_vector,
                                        slope,
                                        -1,
                                    )
                                )
    return hits
