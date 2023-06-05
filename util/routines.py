from util.common import *
from rlbot.utils.structures.ball_prediction_struct import BallPrediction

# This file holds all of the mechanical tasks, called "routines", that the bot can do


class drive:
    def __init__(self, speed, target=None) -> None:
        self.speed = speed
        self.target = target

    def run(self, agent):
        defaultThrottle(agent, self.speed)
        if self.target is not None:
            relative_target = self.target - agent.me.location
            defaultPD(agent, agent.me.local(relative_target))


class atba:
    # An example routine that just drives towards the ball at max speed
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)


class aerial_shot:
    def __init__(self, ball_location, intercept_time, shot_vector, ratio):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.intercept = self.ball_location - (self.shot_vector * 110)
        self.jump_threshold = 400
        self.jump_time = 0
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = max(raw_time_remaining, 0.01)

        car_to_ball = self.ball_location - agent.me.location
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_intercept = self.intercept - agent.me.location
        car_to_intercept_perp = car_to_intercept.cross((0, 0, side_of_shot))
        flat_distance_remaining = car_to_intercept.flatten().magnitude()

        speed_required = flat_distance_remaining / time_remaining
        acceleration_required = backsolve(self.intercept, agent.me, time_remaining, 325)
        local_acceleration_required = agent.me.local(acceleration_required)

        adjustment = (
            car_to_intercept.angle(self.shot_vector) * flat_distance_remaining / 1.57
        )
        adjustment *= (
            cap(
                self.jump_threshold - (acceleration_required[2]),
                0.0,
                self.jump_threshold,
            )
            / self.jump_threshold
        )
        final_target = (
            self.intercept
            if self.jump_time != 0
            else self.intercept + (car_to_intercept_perp.normalize() * adjustment)
        )

        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_final_target = agent.me.local(final_target - agent.me.location)

        agent.line(agent.me.location, self.intercept)
        agent.line(
            self.intercept - Vector3(0, 0, 100),
            self.intercept + Vector3(0, 0, 100),
            [255, 0, 0],
        )
        agent.line(
            final_target - Vector3(0, 0, 100),
            final_target + Vector3(0, 0, 100),
            [0, 255, 0],
        )

        angles = defaultPD(agent, local_final_target)

        if self.jump_time == 0:
            defaultThrottle(agent, speed_required)
            agent.controller.boost = (
                False
                if abs(angles[1]) > 0.3 or agent.me.airborne
                else agent.controller.boost
            )
            agent.controller.handbrake = (
                True if abs(angles[1]) > 2.3 else agent.controller.handbrake
            )

            velocity_required = car_to_intercept / time_remaining
            good_slope = (
                velocity_required[2]
                / cap(abs(velocity_required[0]) + abs(velocity_required[1]), 1, 10000)
                > 0.15
            )
            if (
                good_slope
                and (local_acceleration_required[2]) > self.jump_threshold
                and agent.me.velocity.flatten()
                .normalize()
                .dot(acceleration_required.flatten().normalize())
                > 0.8
            ):
                self.jump_time = agent.time
        else:
            time_since_jump = agent.time - self.jump_time

            if (
                agent.me.airborne
                and local_acceleration_required.magnitude() * time_remaining > 90
            ):
                angles = defaultPD(agent, local_acceleration_required)
                if abs(angles[0]) + abs(angles[1]) < 0.45:
                    agent.controller.boost = True
            else:
                final_target -= Vector3(0, 0, 45)
                local_final_target = agent.me.local(final_target - agent.me.location)
                angles = defaultPD(agent, local_final_target)

            if self.counter == 0 and (
                time_since_jump <= 0.2 and local_acceleration_required[2] > 0
            ):
                agent.controller.jump = True
            elif time_since_jump > 0.2 and self.counter < 3:
                agent.controller.jump = False
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                self.counter += 1
            elif local_acceleration_required[2] > 300 and self.counter == 3:
                agent.controller.jump = True
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                self.counter += 1

        if raw_time_remaining < -0.25:
            agent.set_intent(recovery())
        if not shot_valid(agent, self, 90):
            agent.clear_intent()


class flip:
    def __init__(self, vector, cancel=False, speedflip=False):
        self.vector = vector.normalize()
        self.pitch = abs(self.vector[0]) * -sign(self.vector[0])
        self.yaw = abs(self.vector[1]) * sign(self.vector[1])
        self.cancel = cancel
        self.speedflip = speedflip
        self.time = -1
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time

        if elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed >= 0.15 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif self.speedflip and elapsed < 0.05:
            agent.controller.jump = False
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
            agent.controller.roll = 0
        elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.set_intent(recovery())


class goto:
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    # TODO - slow down if target is inside our turn radius
    def __init__(self, target, vector=None, direction=1):
        self.target = target
        self.vector = vector
        self.direction = direction

    def run(self, agent):
        car_to_target = self.target - agent.me.location
        distance_remaining = car_to_target.flatten().magnitude()

        agent.line(
            self.target - Vector3(0, 0, 500),
            self.target + Vector3(0, 0, 500),
            [255, 0, 255],
        )

        if self.vector != None:
            # See commends for adjustment in jump_shot or aerial for explanation
            side_of_vector = sign(self.vector.cross((0, 0, 1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross((0, 0, side_of_vector)).normalize()
            adjustment = car_to_target.angle(self.vector) * distance_remaining / 3.14
            final_target = self.target + (car_to_target_perp * adjustment)
        else:
            final_target = self.target

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target, self.direction)
        defaultThrottle(agent, 2300, self.direction)

        agent.controller.boost = (
            True if distance_remaining > 1000 else agent.controller.boost
        )
        agent.controller.handbrake = (
            True if abs(angles[1]) > 2.3 else agent.controller.handbrake
        )

        velocity = 1 + agent.me.velocity.magnitude()
        if distance_remaining < 350:
            agent.clear_intent()
        elif (
            abs(angles[1]) < 0.05
            and velocity > 600
            and velocity < 2150
            and distance_remaining / velocity > 2.0
        ):
            agent.set_intent(flip(local_target))
        elif abs(angles[1]) > 2.8 and velocity < 200:
            agent.set_intent(flip(local_target))
        elif agent.me.airborne:
            agent.set_intent(recovery(self.target))


class goto_boost:
    # very similar to goto() but designed for grabbing boost
    # if a target is provided the bot will try to be facing the target as it passes over the boost
    def __init__(self, boost, target=None):
        self.boost = boost
        self.target = target

    def run(self, agent):
        car_to_boost = self.boost.location - agent.me.location
        distance_remaining = car_to_boost.flatten().magnitude()

        agent.line(
            self.boost.location - Vector3(0, 0, 500),
            self.boost.location + Vector3(0, 0, 500),
            [0, 255, 0],
        )

        if self.target != None:
            vector = (self.target - self.boost.location).normalize()
            side_of_vector = sign(vector.cross((0, 0, 1)).dot(car_to_boost))
            car_to_boost_perp = car_to_boost.cross((0, 0, side_of_vector)).normalize()
            adjustment = car_to_boost.angle(vector) * distance_remaining / 3.14
            final_target = self.boost.location + (car_to_boost_perp * adjustment)
            car_to_target = (self.target - agent.me.location).magnitude()
        else:
            adjustment = 9999
            car_to_target = 0
            final_target = self.boost.location.copy()

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 1410)

        agent.controller.boost = self.boost.large if abs(angles[1]) < 0.3 else False
        agent.controller.handbrake = (
            True if abs(angles[1]) > 2.3 else agent.controller.handbrake
        )

        velocity = 1 + agent.me.velocity.magnitude()
        if (
            self.boost.active == False
            or agent.me.boost >= 99.0
            or distance_remaining < 350
        ):
            agent.clear_intent()
        elif agent.me.airborne:
            agent.set_intent(recovery(self.target))
        elif (
            abs(angles[1]) < 0.05
            and velocity > 600
            and velocity < 2150
            and (
                distance_remaining / velocity > 2.0
                or (adjustment < 90 and car_to_target / velocity > 2.0)
            )
        ):
            agent.set_intent(flip(local_target))


class jump_shot:
    def __init__(
        self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300
    ):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.dodge_point = self.ball_location - (self.shot_vector * 173)
        self.ratio = ratio
        self.direction = direction
        self.speed_desired = speed
        self.jump_threshold = 300
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = max(raw_time_remaining, 0.001)

        car_to_ball = self.ball_location - agent.me.location
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = car_to_dodge_point.cross((0, 0, side_of_shot))
        distance_remaining = car_to_dodge_point.magnitude()

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(
            self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650
        )
        local_acceleration_required = agent.me.local(acceleration_required)

        adjustment = (
            car_to_dodge_point.angle(self.shot_vector) * distance_remaining / 2.0
        )
        adjustment *= (
            cap(
                self.jump_threshold - (acceleration_required[2]),
                0.0,
                self.jump_threshold,
            )
            / self.jump_threshold
        )
        final_target = (
            self.dodge_point
            + (car_to_dodge_perp.normalize() * adjustment if not self.jumping else 0)
            + Vector3(0, 0, 50)
        )

        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -850, 850)

        local_final_target = agent.me.local(final_target - agent.me.location)

        agent.line(agent.me.location, self.dodge_point)
        agent.line(
            self.dodge_point - Vector3(0, 0, 100),
            self.dodge_point + Vector3(0, 0, 100),
            [255, 0, 0],
        )
        agent.line(
            final_target - Vector3(0, 0, 100),
            final_target + Vector3(0, 0, 100),
            [0, 255, 0],
        )
        agent.line(agent.ball.location, agent.ball.location + (self.shot_vector * 300))

        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.controller.boost = (
            False
            if abs(angles[1]) > 0.3 or agent.me.airborne
            else agent.controller.boost
        )
        agent.controller.handbrake = (
            True
            if abs(angles[1]) > 2
            or local_final_target.magnitude()
            < find_turn_radius(abs(agent.me.local(agent.me.velocity)[0]))
            * abs(angles[1])
            else agent.controller.handbrake
        )

        if not self.jumping:
            if (
                raw_time_remaining <= 0.0
                or (speed_required - 2300) * time_remaining > 60
                or not shot_valid(agent, self)
            ):
                agent.clear_intent()
                if agent.me.airborne:
                    agent.set_intent(recovery())
            elif (
                local_acceleration_required[2] > self.jump_threshold
                and local_acceleration_required[2]
                > local_acceleration_required.flatten().magnitude()
            ):
                self.jumping = True
        else:
            if (
                (raw_time_remaining > 0.2 and not shot_valid(agent, self, 150))
                or raw_time_remaining <= -0.9
                or (not agent.me.airborne and self.counter > 0)
            ):
                agent.set_intent(recovery())
            elif (
                self.counter == 0
                and local_acceleration_required[2] > 0.0
                and raw_time_remaining > 0.083
            ):
                agent.controller.jump = True
            elif self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -sign(vector[0])
                    self.y = abs(vector[1]) * sign(vector[1]) * self.direction
                    self.dodging = True
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0


class speedflip:
    def __init__(self, direction):
        self.direction = direction
        self.time = -1
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time

        if elapsed < 0.05:
            agent.controller.jump = True
        elif elapsed >= 0.05 and elapsed < 0.15:
            agent.controller.jump = False
            agent.controller.pitch = -1
            agent.controller.yaw = self.direction[1]
            agent.controller.roll = self.direction[0]
        elif elapsed >= 0.15 and elapsed < 0.2:
            agent.controller.jump = True
            agent.controller.pitch = -1
            agent.controller.yaw = self.direction[1]
            agent.controller.roll = self.direction[0]
        else:
            agent.set_intent(recovery())


class kickoff:
    def run(self, agent):
        target = agent.ball.location + Vector3(0, 200 * side(agent.team), 0)

        # Check if the bot is in a slight-offcenter kickoff position
        if abs(agent.me.location.x) > 100:
            # Adjust the target to grab the boost
            target += Vector3(100 * sign(agent.me.location.x), 0, 0)

        local_target = agent.me.local(target - agent.me.location)

        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        if local_target.magnitude() < 650:
            agent.set_intent(
                flip(agent.me.local(agent.foe_goal.location - agent.me.location))
            )


class recovery:
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None, boosting=False):
        self.target = target
        self.boosting = boosting

    def run(self, agent):
        if self.target != None:
            local_target = agent.me.local(
                (self.target - agent.me.location).flatten().normalize()
            )
        else:
            local_target = agent.me.local(agent.me.velocity.flatten().normalize())

        angles = defaultPD(agent, local_target)
        agent.controller.throttle = 1
        agent.controller.boost = self.boosting

        if not agent.me.airborne:
            agent.clear_intent()
        elif not agent.me.doublejumped:
            agent.clear_intent()
            # Wavedash recovery!
            agent.set_intent(wavedash())


class wavedash:
    # this routine will wavedash on recovery!
    def __init__(self):
        self.step = 0

    def run(self, agent):
        if agent.me.velocity.flatten().magnitude() > 100:
            target = agent.me.velocity.flatten().normalize() * 100 + Vector3(0, 0, 50)
        else:
            target = agent.me.forward.flatten() * 100 + Vector3(0, 0, 50)
        local_target = agent.me.local(target)
        defaultPD(agent, local_target)
        if self.step < 6 and not agent.me.airborne:
            self.step += 1
            if self.step < 3:
                agent.controller.jump = True
            else:
                agent.controller.jump = False
        else:
            if not agent.me.airborne or agent.me.doublejumped:
                agent.clear_intent()
            elif agent.me.location.z + agent.me.velocity.z * 0.2 < 5:
                agent.controller.jump = True
                agent.controller.pitch = -1
                agent.controller.yaw = agent.controller.roll = 0
                agent.clear_intent()


class short_shot:
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball = (agent.ball.location - agent.me.location).normalize()
        distance = (agent.ball.location - agent.me.location).magnitude()
        ball_to_target = (self.target - agent.ball.location).normalize()

        ball_to_our_goal = agent.friend_goal.location - agent.ball.location

        relative_velocity = car_to_ball.dot(agent.me.velocity - agent.ball.velocity)
        if relative_velocity != 0.0:
            etaf = eta(agent.me, self.target, car_to_ball, distance)
        else:
            etaf = 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross((0, 0, 1))
        right_vector = car_to_ball.cross((0, 0, -1))
        target_vector = -ball_to_target.clamp(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector * (distance / 2))

        # Some extra adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5000:
            if abs(final_target[0]) < 800:
                final_target[0] = cap(final_target[0], -800, 800)
            else:
                final_target[1] = cap(final_target[1], -5100, 5100)

        agent.line(
            final_target - Vector3(0, 0, 100),
            final_target + Vector3(0, 0, 100),
            [255, 255, 255],
        )

        angles = defaultPD(agent, agent.me.local(final_target - agent.me.location))
        defaultThrottle(
            agent,
            2300 if distance > 1600 else 2300 - cap(1600 * abs(angles[1]), 0, 2050),
        )
        agent.controller.boost = (
            False
            if agent.me.airborne or abs(angles[1]) > 0.3
            else agent.controller.boost
        )
        agent.controller.handbrake = (
            True if abs(angles[1]) > 2.3 else agent.controller.handbrake
        )

        if abs(angles[1]) < 0.05 and (etaf < 0.45 or distance < 150):
            agent.clear_intent()
            agent.set_intent(flip(agent.me.local(car_to_ball)))
        elif distance > 1500:
            agent.set_intent(flip(agent.me.local(car_to_ball)))
        if etaf > 1 or ball_to_our_goal.magnitude() < 500 or agent.rotation_index != 0:
            agent.clear_intent()
