import math
from math import atan2, pi

from util.common import *

# This file holds all of the mechanical tasks, called "routines", that the bot can do

GRAVITY: Vector3 = Vector3(0, 0, -650)

# Aerial constants
MAX_SPEED: float = 2300
BOOST_ACCEL: float = 1060
THROTTLE_ACCEL: float = 200 / 3
BOOST_PER_SECOND: float = 30

# Jump constants

JUMP_SPEED: float = 291.667
JUMP_ACC = 1458.3333
JUMP_MIN_DURATION = 0.025
JUMP_MAX_DURATION = 0.2


class atba:
    # An example routine that just drives towards the ball at max speed
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)


class go_centre:
    # Makes bot go inbetween centre of goal and ball
    # Useful when bot has nothing to do but wants to be in a good position to defend or attack
    def run(self, agent):
        relative_target = (
            Vector3(
                agent.ball.location.x / 3,
                (agent.ball.location.y + 5120 * side(agent.team)) / 3,
                50,
            )
            - agent.me.location
        )
        local_target = agent.me.local(relative_target)
        if relative_target.magnitude() > 350:
            defaultPD(agent, local_target)
            defaultThrottle(agent, 2300)
            angles = defaultPD(agent, agent.me.local(relative_target))
            if abs(angles[1]) > 1.5:
                agent.controller.handbrake = True
                agent.controller.boost = False
            else:
                agent.pop()
        else:
            # If close to target, stop moving and face towards ball
            defaultThrottle(agent, 0)
            relative_target = agent.ball.location - agent.me.location
            angles = defaultPD(agent, agent.me.local(relative_target))
            if abs(angles[1]) > 2.88 and abs(angles[1]) < 3.4:
                agent.push(half_flip())
            else:
                agent.pop()
        # Line between car and desired location
        agent.line(
            Vector3(
                agent.ball.location.x / 2,
                (agent.ball.location.y + 5120 * side(agent.team)) / 2,
                50,
            ),
            agent.me.location,
            [255, 0, 255],
        )


class goto_friendly_goal:
    # Drives towards friendly goal. If touching or over goal line stops moving and faces enemy goal
    def __init__(self):
        self.step = 10

    def run(self, agent):
        if (
            agent.me.location.y > -4800
            if side(agent.team) == -1
            else agent.me.location.y < 4800
        ):
            relative = Vector3(0, 5120 * side(agent.team), 0) - agent.me.location
            defaultPD(agent, agent.me.local(relative))
            angles = defaultPD(agent, agent.me.local(relative))
            if (
                agent.me.location.y > -4750
                if side(agent.team) == -1
                else agent.me.location.y < 4750
            ):
                if abs(angles[1]) > 2.88 and abs(angles[1]) < 3.4:
                    defaultThrottle(agent, -2300)
                    self.step = 10
                else:
                    defaultThrottle(agent, 2300)
                    self.step = 0
        elif self.step == 0:
            agent.push(half_flip())
            self.step = 10


class half_flip:
    def __init__(self):
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        if elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed >= 0.15 and self.counter < 1:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.6:
            agent.controller.jump = True
            agent.controller.pitch = 1
        elif elapsed < 1.4:
            # Rotates and lands
            agent.controller.pitch = -1
            agent.controller.roll = 1
            agent.controller.yaw = 1
        elif not agent.me.airborne:
            # Stops routine when landed
            agent.pop()


class aerial:
    def __init__(
        self,
        ball_location: Vector3,
        intercept_time: float,
        on_ground: bool,
        target: Vector3 = Vector3(0, 0, 0),
    ):
        super().__init__()
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.target = target
        self.jumping = on_ground
        self.time = -1
        self.jump_time = -1
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        T = self.intercept_time - agent.time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * GRAVITY * T**2
        vf = agent.me.velocity + GRAVITY * T
        if self.jumping:
            if self.jump_time == -1:
                jump_elapsed = 0
                self.jump_time = agent.time
            else:
                jump_elapsed = agent.time - self.jump_time
            tau = JUMP_MAX_DURATION - jump_elapsed
            if jump_elapsed == 0:
                vf += agent.me.orientation.up * JUMP_SPEED
                xf += agent.me.orientation.up * JUMP_SPEED * T

            vf += agent.me.orientation.up * JUMP_ACC * tau
            xf += agent.me.orientation.up * JUMP_ACC * tau * (T - 0.5 * tau)

            vf += agent.me.orientation.up * JUMP_SPEED
            xf += agent.me.orientation.up * JUMP_SPEED * (T - tau)

            if jump_elapsed < JUMP_MAX_DURATION:
                agent.controller.jump = True
            elif elapsed >= JUMP_MAX_DURATION and self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif elapsed < 0.3:
                agent.controller.jump = True
            else:
                self.jumping = jump_elapsed <= 0.3
        else:
            agent.controller.jump = 0

        delta_x = self.ball_location - xf
        direction = delta_x.normalize()
        if delta_x.magnitude() > 50:
            defaultPD(agent, agent.me.local(delta_x))
        else:
            if self.target is not None:
                defaultPD(agent, agent.me.local(self.target))
            else:
                defaultPD(agent, agent.me.local(self.ball_location - agent.me.location))

        if JUMP_MAX_DURATION <= elapsed < 0.3 and self.counter == 3:
            agent.controller.roll = 0
            agent.controller.pitch = 0
            agent.controller.yaw = 0
            agent.controller.steer = 0

        if agent.me.forward.angle3D(direction) < 0.3:
            if delta_x.magnitude() > 50:
                agent.controller.boost = 1
                agent.controller.throttle = 0
            else:
                agent.controller.boost = 0
                agent.controller.throttle = cap(0.5 * THROTTLE_ACCEL * T**2, 0, 1)
        else:
            agent.controller.boost = 0
            agent.controller.throttle = 0

        if T <= 0 or not shot_valid(agent, self, threshold=90):
            agent.pop()
            agent.push(recovery())

    def is_viable(self, agent, time: float):
        T = self.intercept_time - time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * GRAVITY * T**2
        vf = agent.me.velocity + GRAVITY * T
        if not agent.me.airborne:
            vf += agent.me.orientation.up * (
                2 * JUMP_SPEED + JUMP_ACC * JUMP_MAX_DURATION
            )
            xf += agent.me.orientation.up * (
                JUMP_SPEED * (2 * T - JUMP_MAX_DURATION)
                + JUMP_ACC * (T * JUMP_MAX_DURATION - 0.5 * JUMP_MAX_DURATION**2)
            )

        delta_x = self.ball_location - xf
        f = delta_x.normalize()
        phi = f.angle3D(agent.me.forward)

        if phi == 0:
            phi = 1
        turn_time = 0.7 * (2 * math.sqrt(phi / 9))

        tau1 = turn_time * cap(1 - 0.3 / phi, 0, 1)
        required_acc = (2 * delta_x.magnitude()) / ((T - tau1) ** 2)
        ratio = required_acc / BOOST_ACCEL
        tau2 = T - (T - tau1) * math.sqrt(1 - cap(ratio, 0, 1))
        velocity_estimate = vf + BOOST_ACCEL * (tau2 - tau1) * f
        boos_estimate = (tau2 - tau1) * 30
        enough_boost = boos_estimate < 0.95 * agent.me.boost
        enough_time = abs(ratio) < 0.9

        in_goal = abs(agent.me.location.y) > 5000

        return (
            velocity_estimate.magnitude() < 0.9 * MAX_SPEED
            and enough_boost
            and enough_time
            and not in_goal
        )


class aerial_shot:
    # Very similar to jump_shot(), but instead designed to hit targets above 300uu
    # ***This routine is a WIP*** It does not currently hit the ball very hard, nor does it like to be accurate above 600uu or so
    def __init__(self, ball_location, intercept_time, shot_vector, ratio):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we hit the ball at
        self.intercept = self.ball_location - (self.shot_vector * 100)
        # dictates when (how late) we jump, much later than in jump_shot because we can take advantage of a double jump
        self.jump_threshold = 584
        # what time we began our jump at
        self.jump_time = 0
        # If we need a second jump we have to let go of the jump button for 3 frames, this counts how many frames we have let go for
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.01, 10.0)

        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_intercept = self.intercept - agent.me.location
        car_to_intercept_perp = car_to_intercept.cross(
            (0, 0, side_of_shot)
        )  # perpendicular
        distance_remaining = car_to_intercept.flatten().magnitude()

        speed_required = distance_remaining / time_remaining
        # When still on the ground we pretend GRAVITY doesn't exist, for better or worse
        acceleration_required = backsolve(
            self.intercept, agent.me, time_remaining, 0 if self.jump_time == 0 else 325
        )
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        adjustment = (
            car_to_intercept.angle(self.shot_vector) * distance_remaining / 1.57
        )  # size of adjustment
        adjustment *= (
            cap(
                self.jump_threshold - (acceleration_required[2]),
                0.0,
                self.jump_threshold,
            )
            / self.jump_threshold
        )  # factoring in how close to jump we are
        # we don't adjust the final target if we are already jumping
        final_target = self.intercept + (
            (car_to_intercept_perp.normalize() * adjustment)
            if self.jump_time == 0
            else 0
        )

        # Some extra adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5000:
            if abs(final_target[0]) < 800:
                final_target[0] = cap(final_target[0], -800, 800)
            else:
                final_target[1] = cap(final_target[1], -5100, 5100)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
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
            if (
                local_acceleration_required[2] > self.jump_threshold
                and local_acceleration_required[2]
                > local_acceleration_required.flatten().magnitude()
            ):
                # Switch into the jump when the upward acceleration required reaches our threshold, hopefully we have aligned already...
                self.jump_time = agent.time
        else:
            time_since_jump = agent.time - self.jump_time

            # While airborne we boost if we're within 30 degrees of our local acceleration requirement
            if (
                agent.me.airborne
                and local_acceleration_required.magnitude() * time_remaining > 100
            ):
                if agent.me.boost > 1:
                    angles = defaultPD(agent, local_acceleration_required)
                    if abs(angles[0]) + abs(angles[1]) < 0.5:
                        agent.controller.boost = True
                else:
                    angles = defaultPD(agent, agent.me.local(self.shot_vector))
            if self.counter == 0 and (
                time_since_jump <= 0.2 and local_acceleration_required[2] > 0
            ):
                # hold the jump button up to 0.2 seconds to get the most acceleration from the first jump
                agent.controller.jump = True
            elif time_since_jump > 0.2 and self.counter < 3:
                # Release the jump button for 3 ticks
                agent.controller.jump = False
                self.counter += 1
            elif local_acceleration_required[2] > 300 and self.counter == 3:
                # the acceleration from the second jump is instant, so we only do it for 1 frame
                agent.controller.jump = True
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                self.counter += 1

        if (
            raw_time_remaining < -0.25
            or not shot_valid(agent, self)
            or (
                agent.rotation_index != 0
                and local_acceleration_required[2] < self.jump_threshold / 5
                and self.jump_time == 0
            )
        ):
            agent.pop()
            agent.push(recovery())


class flip:
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False, boosting=False):
        self.vector = vector.normalize()
        self.pitch = -self.vector[0]
        self.yaw = self.vector[1]
        self.cancel = cancel
        self.boosting = boosting
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        if elapsed < 0.1:
            agent.controller.jump = True
        elif elapsed >= 0.1 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.25 or (not self.cancel and elapsed < 0.85):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.pop()
            agent.push(recovery(None, self.boosting))
        if self.boosting:
            agent.controller.boost = (
                agent.me.local(agent.me.forward).angle3D(self.vector) < math.pi
            )


class goto:
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    # TODO - slow down if target is inside our turn radius
    def __init__(self, target, vector=None, direction=1):
        self.target = target
        self.vector = vector
        self.direction = direction

    def update(self, target, vector=None, direction=1):
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
        if abs(agent.me.location[1]) > 5100:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target, self.direction)
        defaultThrottle(agent, 2300, self.direction)

        agent.controller.boost = (
            False if not is_on_wall(agent.me.location) else agent.controller.boost
        )
        agent.controller.handbrake = (
            True if abs(angles[1]) > 2.3 else agent.controller.handbrake
        )

        velocity = 1 + agent.me.velocity.magnitude()
        if distance_remaining < 500:
            agent.pop()
        elif (
            abs(angles[1]) < 0.05
            and velocity > 600
            and velocity < 2150
            and distance_remaining / velocity > 2.0
            and not is_on_wall(agent.me.location)
        ):
            agent.push(speed_flip(self.direction))
        elif (
            abs(angles[1]) > 2.8
            and velocity < 200
            and not is_on_wall(agent.me.location)
        ):
            agent.push(flip(local_target, True))
        elif agent.me.airborne:
            agent.push(recovery(self.target))


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
            adjustment = car_to_boost.angle(vector) * distance_remaining / 3.5
            final_target = self.boost.location + (car_to_boost_perp * adjustment)
            if on_wall(agent.me.location):
                final_target = final_target.flatten()
            # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
            if abs(agent.me.location[1]) > 5100:
                final_target[0] = cap(final_target[0], -800, 800)
                final_target[1] = cap(final_target[1], -5000, 5000)

            if abs(final_target[0]) > 4096:
                final_target[0] = (4000 + distance_to_wall(agent.me.location)) * sign(
                    final_target[0]
                )
            if abs(final_target[1]) > 5120:
                final_target[1] = (5000 + distance_to_wall(agent.me.location)) * sign(
                    final_target[1]
                )
            car_to_target = (self.target - agent.me.location).magnitude()
        else:
            adjustment = 9999
            car_to_target = 0
            final_target = self.boost.location

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5100:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        agent.controller.boost = self.boost.large if abs(angles[1]) < 0.3 else False
        agent.controller.handbrake = (
            True if abs(angles[1]) > 1.6 else agent.controller.handbrake
        )

        velocity = 1 + agent.me.velocity.magnitude()
        if (
            self.boost.active == False
            or agent.me.boost >= 99.0
            or distance_remaining < 350
            or agent.rotation_index == 0
        ):
            agent.pop()
        elif agent.me.airborne:
            agent.push(recovery(self.target))
        elif (
            abs(angles[1]) < 0.05
            and velocity > 600
            and velocity < 2150
            and distance_remaining / velocity > 2.5
        ):
            agent.push(flip(local_target))


class goto_pad:
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
        else:
            final_target = self.boost.location

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5100:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        agent.controller.boost = self.boost.large if abs(angles[1]) < 0.3 else False
        agent.controller.handbrake = (
            True if abs(angles[1]) > 2.3 else agent.controller.handbrake
        )

        if (
            self.boost.active == False
            or agent.me.boost >= 99.0
            or distance_remaining < 800
            or distance_remaining > 1200
            or agent.rotation_index == 0
        ):
            agent.pop()
        elif agent.me.airborne:
            agent.push(recovery(self.target))


class jump_shot:
    # Hits a target point at a target time towards a target direction
    # Target must be no higher than 300uu unless you're feeling lucky
    # TODO - speed
    def __init__(
        self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300
    ):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we dodge at
        # 173 is the 93uu ball radius + a bit more to account for the car's hitbox
        self.dodge_point = self.ball_location - (self.shot_vector * 170)
        # Ratio is how aligned the car is. Low ratios (<0.5) aren't likely to be hit properly
        self.ratio = ratio
        # whether the car should attempt this backwards
        self.direction = direction
        # Intercept speed not implemented
        self.speed_desired = speed
        # controls how soon car will jump based on acceleration required. max 584
        # bigger = later, which allows more time to align with shot vector
        # smaller = sooner
        self.jump_threshold = 384
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = car_to_dodge_point.cross(
            (0, 0, side_of_shot)
        )  # perpendicular
        distance_remaining = car_to_dodge_point.magnitude()

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(
            self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650
        )
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump

        adjustment = (
            car_to_dodge_point.angle(self.shot_vector) * distance_remaining / 2.0
        )  # size of adjustment
        adjustment *= (
            cap(
                self.jump_threshold - (acceleration_required[2]),
                0.0,
                self.jump_threshold,
            )
            / self.jump_threshold
        )  # factoring in how close to jump we are
        # we don't adjust the final target if we are already jumping
        final_target = (
            self.dodge_point
            + ((car_to_dodge_perp.normalize() * adjustment) if not self.jumping else 0)
            + Vector3(0, 0, 50)
        )
        # Ensuring our target isn't too close to the sides of the field, where our car would get messed up by the radius of the curves

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5100:
            final_target[0] = cap(final_target[0], -800, 800)
            final_target[1] = cap(final_target[1], -5000, 5000)

        distance_from_goal = (
            self.ball_location - agent.friend_goal.location
        ).magnitude()

        if distance_from_goal < 6000:
            if abs(final_target[0]) > 4096:
                final_target[0] = (4000 + distance_to_wall(agent.me.location)) * sign(
                    final_target[0]
                )
            if abs(final_target[1]) > 5120:
                final_target[1] = (5000 + distance_to_wall(agent.me.location)) * sign(
                    final_target[1]
                )

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
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

        # Calling our drive utils to get us going towards the final target
        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.line(
            agent.me.location,
            agent.me.location + (self.shot_vector * 200),
            [255, 255, 255],
        )

        agent.controller.boost = (
            False
            if abs(angles[1]) > 0.3 or agent.me.airborne
            else agent.controller.boost
        )
        agent.controller.handbrake = (
            True
            if abs(angles[1]) > 2.3 and self.direction == 1
            else agent.controller.handbrake
        )

        if not self.jumping:
            if (
                raw_time_remaining <= 0.0
                or (speed_required - 2300) * time_remaining > 45
                or not shot_valid(agent, self)
                or (
                    agent.rotation_index != 0
                    and local_acceleration_required[2] < self.jump_threshold / 5
                )
            ):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif (
                local_acceleration_required[2] > self.jump_threshold
                and local_acceleration_required[2]
                > local_acceleration_required.flatten().magnitude()
            ):
                # Switch into the jump when the upward acceleration required reaches our threshold, and our lateral acceleration is negligible
                self.jumping = True
        else:
            if (
                (raw_time_remaining > 0.2 and not shot_valid(agent, self, 60))
                or raw_time_remaining <= -0.9
                or (not agent.me.airborne and self.counter > 0)
            ):
                agent.pop()
                agent.push(recovery())
            elif (
                self.counter == 0
                and local_acceleration_required[2] > 0.0
                and raw_time_remaining > 0.083
            ):
                # Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                # make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                # dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -sign(vector[0])
                    self.y = abs(vector[1]) * sign(vector[1])
                    self.dodging = True
                # simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0


class wall_shot:
    # Hits a target point at a target time towards a target direction
    # Target must be no higher than 300uu unless you're feeling lucky
    # TODO - speed
    def __init__(
        self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300
    ):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we dodge at
        # 173 is the 93uu ball radius + a bit more to account for the car's hitbox
        self.dodge_point = self.ball_location - (self.shot_vector * 170)
        # Ratio is how aligned the car is. Low ratios (<0.5) aren't likely to be hit properly
        self.ratio = ratio
        # whether the car should attempt this backwards
        self.direction = direction
        # Intercept speed not implemented
        self.speed_desired = speed
        # controls how soon car will jump based on acceleration required. max 584
        # bigger = later, which allows more time to align with shot vector
        # smaller = sooner
        self.jump_threshold = 384
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.ball_location[0])

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = car_to_dodge_point.cross(
            (0, 0, side_of_shot)
        )  # perpendicular
        distance_remaining = car_to_dodge_point.magnitude()

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(
            self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650
        )
        local_acceleration_required = agent.me.local(acceleration_required)

        height_difference = abs(agent.me.location[2] - self.ball_location[2])

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        adjustment = distance_remaining * 2  # size of adjustment
        # we don't adjust the final target if we are already jumping
        final_target = (
            self.dodge_point
            + (car_to_dodge_perp.normalize() * adjustment if not self.jumping else 0)
            + Vector3(0, 0, 50)
        )
        # Ensuring our target isn't too close to the sides of the field, where our car would get messed up by the radius of the curves

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5050 and abs(agent.me.location[0]) < 900:
            if abs(final_target[0]) < 750:
                final_target[0] = cap(final_target[0], -700, 700)
            else:
                final_target[1] = cap(final_target[1], -5000, 5000)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
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

        # Calling our drive utils to get us going towards the final target
        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.line(
            agent.me.location,
            agent.me.location + (self.shot_vector * 200),
            [255, 255, 255],
        )

        agent.controller.boost = (
            False
            if abs(angles[1]) > 0.3 or agent.me.airborne
            else agent.controller.boost
        )
        agent.controller.handbrake = (
            True
            if abs(angles[1]) > 2.3 and self.direction == 1
            else agent.controller.handbrake
        )

        if not self.jumping:
            if (
                raw_time_remaining <= 0.0
                or (speed_required - 2300) * time_remaining > 45
                or not shot_valid(agent, self)
                or agent.rotation_index != 0
            ):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
                agent.pop()
                if agent.me.airborne:
                    agent.push(recovery())
            elif (
                local_acceleration_required[2] > self.jump_threshold
                and local_acceleration_required[2]
                > local_acceleration_required.flatten().magnitude()
                and distance_remaining < 800
                and height_difference < 500
            ):
                # Switch into the jump when the upward acceleration required reaches our threshold, and our lateral acceleration is negligible
                self.jumping = True
        else:
            if (
                (raw_time_remaining > 0.2 and not shot_valid(agent, self, 60))
                or raw_time_remaining <= -0.9
                or (not agent.me.airborne and self.counter > 0)
                or agent.rotation_index != 0
            ):
                agent.pop()
                agent.push(recovery())
            elif (
                self.counter == 0
                and local_acceleration_required[2] > 0.0
                and raw_time_remaining > 0.083
            ):
                # Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
                agent.controller.boost = True
            elif self.counter < 3:
                # make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                agent.controller.boost = True
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                # dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -sign(vector[0])
                    self.y = abs(vector[1]) * sign(vector[1]) * self.direction
                    self.dodging = True
                # simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0


class kickoff2:
    # A simple 1v1 kickoff that just drives up behind the ball and dodges
    # misses the boost on the slight-offcenter kickoffs haha
    def run(self, agent):
        target = agent.ball.location + Vector3(0, 200 * side(agent.team), 0)
        local_target = agent.me.local(target - agent.me.location)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)
        if local_target.magnitude() < 650:
            # flip towards opponent goal
            agent.set_intent(
                flip(agent.me.local(agent.foe_goal.location - agent.me.location))
            )


class kickoff:
    def __init__(self, x):
        self.time_of_kickoff = -1
        self.corner_kickoff = abs(x) > 1500
        self.straight_kickoff = abs(x) < 100
        self.side = sign(x + 1)

    def run(self, agent):
        if self.time_of_kickoff == -1:
            self.time_of_kickoff = agent.time
        elapsed = agent.time - self.time_of_kickoff

        car_to_ball = agent.ball.location - agent.me.location
        car_to_ball_angle = math.atan2(car_to_ball.y, car_to_ball.x)
        car_orientation_angle = math.atan2(agent.me.forward.y, agent.me.forward.x)
        angle_difference = abs(car_to_ball_angle - car_orientation_angle)

        # Check if the car is aligned with the center of the ball
        is_aligned = angle_difference < 0.5

        corner_kickoff = abs(agent.me.location.x) > 1500
        straight_kickoff = abs(agent.me.location.x) < 100
        team = -side(agent.team)

        steer = self.side * team if corner_kickoff else -self.side * team
        local_car_to_ball = agent.me.local(car_to_ball)

        speed_flip_vector = Vector3(1 / math.sqrt(2), -steer / math.sqrt(2), 0)

        if not agent.kickoff_flag:
            agent.pop()

        if self.corner_kickoff:
            if elapsed < 0.23:
                agent.controller.steer = steer / 4
                defaultThrottle(agent, 2300)
            elif elapsed < 0.3 and is_aligned:
                agent.push(flip(speed_flip_vector, True, True))
            elif elapsed < 2:
                agent.push(flip(local_car_to_ball))
        elif self.straight_kickoff:
            if elapsed < 0.4:
                agent.controller.steer = steer / 4
                defaultThrottle(agent, 2300)
            elif elapsed < 0.5 and is_aligned:
                agent.push(flip(speed_flip_vector, True, True))
            elif elapsed < 2.2:
                defaultPD(agent, local_car_to_ball)
                defaultThrottle(agent, 2300)
                agent.controller.handbrake = True
            elif elapsed < 3:
                agent.push(flip(local_car_to_ball))
        else:
            if elapsed < 0.3:
                agent.controller.steer = steer / 4
                defaultThrottle(agent, 2300)
            elif elapsed < 0.5 and is_aligned:
                agent.push(flip(speed_flip_vector, True, True))
            elif elapsed < 2:
                defaultPD(agent, local_car_to_ball)
                defaultThrottle(agent, 2300)
                agent.controller.handbrake = True
            elif elapsed < 3:
                agent.push(flip(local_car_to_ball))


class recovery:
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None, boosting=False):
        self.target = target
        self.boosting = boosting

    def run(self, agent):
        if self.target != None:
            local_target = agent.me.local((self.target - agent.me.location).flatten())
        else:
            local_target = agent.me.local(agent.me.velocity.flatten())

        angles = defaultPD(agent, local_target)
        agent.controller.throttle = 1
        agent.controller.boost = self.boosting

        angles_magnitude = angles[0] + angles[1] + angles[2]

        if angles_magnitude < 1 and not agent.me.doublejumped:
            agent.pop()
            # Wavedash recovery!
            agent.push(wavedash())

        if not agent.me.airborne:
            agent.pop()


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
            if agent.me.location.z + agent.me.velocity.z * 0.2 < 5:
                agent.controller.jump = True
                agent.controller.pitch = -1
                agent.controller.yaw = agent.controller.roll = 0
                agent.pop()
            elif not agent.me.airborne or agent.me.doublejumped:
                agent.pop()


class pop_up:
    # Hits a target point at a target time towards a target direction
    # Target must be no higher than 300uu unless you're feeling lucky
    # TODO - speed
    def __init__(
        self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300
    ):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # Ratio is how aligned the car is. Low ratios (<0.5) aren't likely to be hit properly
        self.ratio = ratio
        # whether the car should attempt this backwards
        self.direction = direction
        # Intercept speed not implemented
        self.speed_desired = speed
        # controls how soon car will jump based on acceleration required. max 584
        # bigger = later, which allows more time to align with shot vector
        # smaller = sooner
        self.jump_threshold = 384
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_point = self.ball_location - agent.me.location
        car_to_perp = car_to_point.cross((0, 0, side_of_shot))  # perpendicular
        distance_remaining = car_to_point.magnitude()

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(
            self.ball_location, agent.me, time_remaining, 0 if not self.jumping else 650
        )
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector

        added_wall_adjustment = cap(4 - distance_to_wall(agent.me.location) / 500, 0, 4)

        adjustment = (
            car_to_point.angle(self.shot_vector) * distance_remaining / 2.0
            + added_wall_adjustment
        )  # size of adjustment
        adjustment *= (
            cap(
                self.jump_threshold - (acceleration_required[2]),
                0.0,
                self.jump_threshold,
            )
            / self.jump_threshold
        )  # factoring in how close to jump we are
        # we don't adjust the final target if we are already jumping
        final_target = (
            self.ball_location
            + ((car_to_perp.normalize() * adjustment) if not self.jumping else 0)
            + Vector3(0, 0, 50)
        )
        # Ensuring our target isn't too close to the sides of the field, where our car would get messed up by the radius of the curves

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5100:
            final_target[0] = cap(final_target[0], -800, 800)
            final_target[1] = cap(final_target[1], -5000, 5000)

        distance_from_goal = (
            self.ball_location - agent.friend_goal.location
        ).magnitude()

        if distance_from_goal < 6000:
            if abs(final_target[0]) > 4096:
                final_target[0] = (4096 + distance_to_wall(agent.me.location)) * sign(
                    final_target[0]
                )
            if abs(final_target[1]) > 5120:
                final_target[1] = (5120 + distance_to_wall(agent.me.location)) * sign(
                    final_target[1]
                )

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.ball_location)
        agent.line(
            self.ball_location - Vector3(0, 0, 100),
            self.ball_location + Vector3(0, 0, 100),
            [255, 0, 0],
        )
        agent.line(
            final_target - Vector3(0, 0, 100),
            final_target + Vector3(0, 0, 100),
            [0, 255, 0],
        )

        # Calling our drive utils to get us going towards the final target
        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.line(
            agent.me.location,
            agent.me.location + (self.shot_vector * 200),
            [255, 255, 255],
        )

        agent.controller.boost = (
            False
            if abs(angles[1]) > 0.3 or agent.me.airborne
            else agent.controller.boost
        )
        agent.controller.handbrake = (
            True
            if abs(angles[1]) > 2.3 and self.direction == 1
            else agent.controller.handbrake
        )

        if (
            raw_time_remaining <= 0.0
            or (speed_required - 2300) * time_remaining > 45
            or not shot_valid(agent, self)
            or agent.rotation_index != 0
        ):
            # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
            agent.pop()
            if agent.me.airborne:
                agent.push(recovery())


class short_shot:
    # This routine drives towards the ball and attempts to hit it towards a given target
    # It does not require ball prediction and kinda guesses at where the ball will be on its own
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball = (agent.ball.location - agent.me.location).normalize()
        distance = (agent.ball.location - agent.me.location).magnitude()
        ball_to_target = (self.target - agent.ball.location).normalize()

        ball_to_our_goal = agent.friend_goal.location - agent.ball.location

        relative_velocity = car_to_ball.dot(agent.me.velocity - agent.ball.velocity)
        if relative_velocity != 0.0:
            eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
        else:
            eta = 1.5

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
        agent.controller.boost = False
        agent.controller.handbrake = (
            True if abs(angles[1]) > 2.3 else agent.controller.handbrake
        )

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.pop()
        elif distance > 350:
            agent.push(goto(ball_to_our_goal))
        if (
            eta > 1
            or (ball_to_our_goal.magnitude() < 500 and distance < 500)
            or agent.rotation_index != 0
        ):
            agent.pop()


class center_ball:
    def __init__(self, target_location):
        self.target = target_location

    def run(self, agent):
        ball_to_target = self.target - agent.ball.location
        desired_ball_location = self.target - ball_to_target.normalize() * 200
        agent.set_intent(goto(desired_ball_location, agent.ball.velocity))


class save:
    def __init__(self):
        self.eta = 0

    def run(self, agent):
        car_to_ball = (agent.ball.location - agent.me.location).normalize()
        distance = (agent.ball.location - agent.me.location).magnitude()

        ball_to_our_goal = (
            agent.ball.location - agent.friend_goal.location
        ).normalize()

        relative_velocity = car_to_ball.dot(agent.me.velocity - agent.ball.velocity)
        if relative_velocity != 0.0:
            self.eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
        else:
            self.eta = 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross((0, 0, 1))
        right_vector = car_to_ball.cross((0, 0, -1))
        target_vector = -ball_to_our_goal.clamp(left_vector, right_vector)
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

        if abs(angles[1]) < 0.05 and (self.eta < 0.45 or distance < 150):
            agent.pop()
            agent.push(flip(agent.me.local(car_to_ball)))
        if self.eta > 1 or agent.rotation_index != 0:
            agent.pop()


# class demo:
#     def __init__(self, target):
#         self.target = target

#     def run(self, agent):
#         distance = (self.target.location - agent.me.location).magnitude()
#         car_to_target = (
#             self.target.location
#             + self.target.velocity * (distance / 2000)
#             - agent.me.location
#         )

#         angles = defaultPD(agent, agent.me.local(car_to_target))
#         defaultThrottle(
#             agent,
#             2300 if distance < 500 else 2300 - cap(1600 * abs(angles[1]), 0, 2050),
#         )
#         agent.controller.boost = False if agent.me.airborne else agent.controller.boost
#         agent.controller.handbrake = (
#             True if abs(angles[1]) > 2.0 else agent.controller.handbrake
#         )

#         if (
#             self.target.location.z > 100
#             and distance < 1000
#             and abs(self.target.location.x) < 4000
#             and abs(self.target.location.y) < 5000
#         ):
#             agent.pop()
#             agent.controller.jump = True
#         elif self.target.demolished or not agent.me.supersonic:
#             agent.pop()


class align_in_goal:
    def __init__(self):
        self.backwards = False

    def run(self, agent):
        my_ball_distance = (
            agent.friend_goal.location - agent.ball.location
        ).magnitude()
        ball_too_close = (
            my_ball_distance < 1500
            or (
                agent.ball.location + agent.ball.velocity - agent.me.location
            ).magnitude()
            < 2000
        )

        friend_goal_to_ball = (
            (agent.friend_goal.location - agent.ball.location).flatten().normalize()
        )
        ideal_position = (
            agent.friend_goal.location - friend_goal_to_ball * 700
        ).flatten()
        ideal_position_to_me = ideal_position - agent.me.location
        ideal_distance = ideal_position_to_me.magnitude()

        me_to_goal = (agent.friend_goal.location - agent.me.location).flatten()

        relative_target = agent.ball.location - agent.me.location
        distance = me_to_goal.magnitude()

        ball_distance = (agent.friend_goal.location - agent.ball.location).magnitude()

        agent.line(
            ideal_position - Vector3(0, 0, 100),
            ideal_position + Vector3(0, 0, 100),
            [0, 0, 0],
        )

        if distance < 700 and not self.backwards:
            defaultPD(agent, agent.me.local(relative_target))
            agent.controller.throttle = 1
            self.backwards = False
        else:
            self.backwards = True
        if distance > 400 and self.backwards:
            defaultPD(agent, agent.me.local(me_to_goal), -1)
            agent.controller.throttle = -1
            self.backwards = True

        if self.backwards and distance < 400:
            agent.pop()

        if ideal_distance < 50 or distance > 900 or ball_too_close:
            agent.pop()


class dribble:
    def __init__(self, target):
        self.target = target
        self.jumping = False
        self.step = 0
        self.eta = 0

    def run(self, agent):
        me_to_goal = (agent.me.location - self.target).normalize()
        balance_spot = agent.me.location + me_to_goal * 20

        ball_to_spot = agent.ball.location - balance_spot
        local_ball_offset = agent.me.local(ball_to_spot)
        distance = ball_to_spot.flatten().magnitude()

        ball_speed = agent.ball.velocity.flatten().magnitude()

        defaultPD(agent, local_ball_offset)
        if self.step == 0:
            car_to_ball = (agent.ball.location - agent.me.location).normalize()

            ball_to_our_goal = (
                agent.ball.location - agent.friend_goal.location
            ).normalize()

            relative_velocity = car_to_ball.dot(agent.me.velocity - agent.ball.velocity)
            if relative_velocity != 0.0:
                self.eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
            else:
                self.eta = 1.5

            # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
            left_vector = car_to_ball.cross((0, 0, 1))
            right_vector = car_to_ball.cross((0, 0, -1))
            target_vector = -ball_to_our_goal.clamp(left_vector, right_vector)
            final_target = agent.ball.location + (target_vector * (distance / 2))

            # Some extra adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
            if abs(agent.me.location[1]) > 5000:
                if abs(final_target[0]) < 800:
                    final_target[0] = cap(final_target[0], -800, 800)
                else:
                    final_target[1] = cap(final_target[1], -5100, 5100)

            angles = defaultPD(agent, agent.me.local(final_target - agent.me.location))
            defaultThrottle(agent, cap(ball_speed + distance + 800, 0, 2300))
            agent.controller.boost = (
                False
                if agent.me.airborne or abs(angles[1]) > 0.3
                else agent.controller.boost
            )
            agent.controller.handbrake = (
                True if abs(angles[1]) > 2.3 else agent.controller.handbrake
            )
            if agent.ball.location.z > 100 and distance < 500:
                self.step = 1
        if self.step == 1:
            defaultThrottle(agent, 2300)
            if distance < 50 and agent.ball.location.z > 80:
                self.step = 2
            elif agent.ball.location.z < 100:
                agent.pop()
        if self.step > 1:
            agent.controller.steer = cap(local_ball_offset[1] / 50, -1, 1)
            defaultThrottle(agent, cap(local_ball_offset[0] * 2 + ball_speed, 0, 2300))
            if agent.ball.location.z < 50 or distance > 200:
                agent.pop()
        if agent.me.airborne or agent.me.location.z > 500 or agent.rotation_index != 0:
            agent.pop()


class steal_boost:
    # slightly tweaked version of GoslingUtils goto_boost class
    # very similar to goto() but designed for grabbing boost
    # if a target is provided the bot will try to be facing the target as it passes over the boost
    def __init__(self, boost, target=None):
        self.boost = boost
        self.target = target
        self.demo = None
        self.attempted_demo = False

    def run(self, agent):
        if self.boost is None:
            boost_cpy = agent.boosts[:]
            if agent.team == 0:
                boosts = [boost_cpy[18], boost_cpy[30], boost_cpy[15], boost_cpy[29]]
            else:
                boosts = [boost_cpy[18], boost_cpy[15], boost_cpy[3], boost_cpy[4]]
            boosts.sort(
                key=lambda boost: (
                    agent.me.location + (2 * agent.me.velocity) - boost.location
                ).magnitude()
            )
            found = False
            for bp in boosts:
                if bp.active:
                    found = True
                    self.boost = bp
            if not found:
                agent.pop()

        if self.boost is not None:
            agent.line(
                self.boost.location - Vector3(0, 0, 500),
                self.boost.location + Vector3(0, 0, 500),
                [0, 255, 0],
            )
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
                car_to_boost_perp = car_to_boost.cross(
                    (0, 0, side_of_vector)
                ).normalize()
                adjustment = car_to_boost.angle(vector) * distance_remaining / 3.14
                final_target = self.boost.location + (car_to_boost_perp * adjustment)
                car_to_target = (self.target - agent.me.location).magnitude()
            else:
                adjustment = 9999
                car_to_target = 0
                final_target = self.boost.location

            # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
            # if abs(agent.me.location[1]) > 5120: final_target[0] = cap(final_target[0], -750, 750)
            if in_goal_area(agent):
                final_target[0] = cap(final_target[0], -750, 750)
                final_target[1] = cap(final_target[1], -5050, 5050)

            local_target = agent.me.local(final_target - agent.me.location)

            angles = defaultPD(agent, local_target)
            defaultThrottle(agent, 2300)

            agent.controller.boost = self.boost.large if abs(angles[1]) < 0.3 else False
            agent.controller.handbrake = (
                True if abs(angles[1]) > 2.3 else agent.controller.handbrake
            )

            velocity = 1 + agent.me.velocity.magnitude()

            """
            demo_coming, democar = detect_demo(agent)
            if demo_coming:
                agent.push(avoid_demo(democar))
            """

            go, index = demo_rotation(agent)
            if are_no_bots_back(agent) or friends_ahead_of_ball(agent) > 0:
                agent.pop()

            elif go and not self.attempted_demo and friends_ahead_of_ball(agent) == 0:
                agent.push(demo(index))
                self.boost = None
                self.attempted_demo = True

            elif (
                self.boost.active == False
                or agent.me.boost >= 99.0
                or distance_remaining < 350
            ):
                agent.pop()
            elif agent.me.airborne:
                agent.push(recovery(self.target))
            elif (
                abs(angles[1]) < 0.05
                and velocity > 600
                and velocity < 2150
                and (
                    distance_remaining / velocity > 2.0
                    or (adjustment < 90 and car_to_target / velocity > 2.0)
                )
            ):
                if abs(agent.controller.yaw) < 0.2:
                    agent.push(flip(local_target))


class demo:
    def __init__(self, target):
        self.target_index = target
        self.target = target

    def run(self, agent):
        car = opponent_car_by_index(agent, self.target_index)

        if car is None:
            car = self.target
        distance_to_target = (agent.me.location - car.location).magnitude()
        velocity = (agent.me.velocity).magnitude()
        velocity_needed = 2200 - velocity
        time_boosting_required = velocity_needed / 991.666
        boost_required = 33.3 * time_boosting_required
        distance_required = velocity * time_boosting_required + 0.5 * 991.666 * (
            time_boosting_required**2
        )

        if velocity < 1:
            velocity = 1
        time_to_target = distance_to_target / velocity
        local_target = agent.me.local(
            car.location + (car.velocity * time_to_target) - agent.me.location
        )
        if abs(agent.me.location[1]) > 5120 - 42.10:
            local_target[0] = cap(local_target[0], -750, 750)

        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        agent.line(
            car.location - Vector3(0, 0, 500),
            car.location + Vector3(0, 0, 500),
            [255, 0, 0],
        )

        # when to stop
        if are_no_bots_back(agent) or friends_ahead_of_ball(agent) > 0:
            agent.pop()
        if car.location[1] * side(agent.team) > -3000 or car.location[2] > 200:
            agent.pop()
        elif agent.me.airborne:
            agent.pop()
            agent.push(recovery())
        elif velocity < 2200:
            if agent.me.boost < boost_required:
                agent.pop()
            elif distance_required > distance_to_target:
                agent.pop()


class goto_kickoff:
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    # TODO - slow down if target is inside our turn radius
    def __init__(self, target, vector=None, direction=1, margin=350):
        self.target = target
        self.vector = vector
        self.direction = direction
        self.margin = margin

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
        if abs(agent.me.location[1] > 5150):
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(Vector3(final_target - agent.me.location))

        angles = defaultPD(agent, local_target, self.direction)
        defaultThrottle(agent, 2300, self.direction)

        velocity = 1 + agent.me.velocity.magnitude()
        botToTargetAngle = atan2(
            agent.ball.location[1] - agent.me.location[1],
            agent.ball.location[0] - agent.me.location[0],
        )
        yaw2 = atan2(agent.me.orientation[1][0], agent.me.orientation[0][0])
        if distance_remaining < self.margin:
            agent.pop()
        elif (
            abs(angles[1]) < 0.05
            and velocity > 600
            and velocity < 2150
            and distance_remaining / velocity > 2.0
        ):
            agent.push(flip(local_target))
        elif (
            distance_remaining / velocity > 2
            and velocity > 1000
            and not agent
            and abs(botToTargetAngle + yaw2) < 0.1
        ):
            if agent.controller.yaw < 0.2:
                agent.push(boost_wave_dash())
        elif agent.me.airborne:
            agent.push(recovery(self.target))


class wave_dash:
    def __init__(self):
        self.start = -1
        self.landing_time = -1

    def run(self, agent):
        agent.controller.throttle = 1
        if self.start == -1:
            self.start = agent.time
            elapsed = 0
            self.target = agent.me.forward + agent.me.up
            self.target.normalize()
        else:
            elapsed = agent.time - self.start
        agent.line(agent.me.location, agent.me.location + (self.target * 200))
        if elapsed < 0.05:
            agent.controller.jump = True
        elif elapsed < 0.15:
            agent.controller.jump = False
            # defaultPD(agent, self.target)
            up = agent.me.local(Vector3(0, 0, 1))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(
                    self.target[2], self.target[0]
                ),  # angle required to pitch towards target
                math.atan2(
                    self.target[1], self.target[0]
                ),  # angle required to yaw towards target
                math.atan2(up[1], up[2]),
            ]
            agent.controller.pitch = steerPD(
                target_angles[0], agent.me.angular_velocity[1] / 4
            )

        elif agent.me.airborne and 18 < agent.me.location[2] < 45:
            agent.controller.jump = True
            agent.controller.pitch = -1
            agent.controller.handbrake = True
        elif not agent.me.airborne:
            if self.landing_time == -1:
                self.landing_time = agent.time
            if agent.time - self.landing_time < 0.18:
                agent.controller.handbrake = True
            else:
                agent.pop()
        if elapsed > 2:
            agent.pop()
            if agent.me.airborne:
                agent.push(recovery())


class boost_wave_dash:
    def __init__(self):
        self.start = -1
        self.landing_time = -1

    def run(self, agent):
        agent.controller.throttle = 1
        if self.start == -1:
            self.start = agent.time
            elapsed = 0
            self.target = agent.me.forward - agent.me.up
            self.target.normalize()
        else:
            elapsed = agent.time - self.start
        agent.line(agent.me.location, agent.me.location + (self.target * 200))
        if elapsed < 0.10:
            agent.controller.jump = True
            agent.controller.boost = True
            up = agent.me.local(Vector3(0, 0, 1))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(
                    self.target[2], self.target[0]
                ),  # angle required to pitch towards target
                math.atan2(
                    self.target[1], self.target[0]
                ),  # angle required to yaw towards target
                math.atan2(up[1], up[2]),
            ]
            agent.controller.pitch = steerPD(
                target_angles[0], agent.me.angular_velocity[1] / 4
            )
        elif elapsed < 0.25:
            agent.controller.jump = False
            agent.controller.boost = True
            up = agent.me.local(Vector3(0, 0, 1))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(
                    self.target[2], self.target[0]
                ),  # angle required to pitch towards target
                math.atan2(
                    self.target[1], self.target[0]
                ),  # angle required to yaw towards target
                math.atan2(up[1], up[2]),
            ]
            agent.controller.pitch = steerPD(
                target_angles[0], agent.me.angular_velocity[1] / 4
            )
        elif elapsed < 0.66:
            agent.controller.boost = True if agent.me.forward[2] < 0 else False
            self.target = agent.me.forward + agent.me.up
            up = agent.me.local(Vector3(0, 0, 1))  # where "up" is in local coordinates
            target_angles = [
                math.atan2(
                    self.target[2], self.target[0]
                ),  # angle required to pitch towards target
                math.atan2(
                    self.target[1], self.target[0]
                ),  # angle required to yaw towards target
                math.atan2(up[1], up[2]),
            ]
            agent.controller.pitch = steerPD(
                target_angles[0], agent.me.angular_velocity[1] / 4
            )
        elif agent.me.airborne and 18 < agent.me.location[2] < 45:
            agent.controller.jump = True
            agent.controller.pitch = -1
            agent.controller.handbrake = True
            agent.controller.boost = True
        elif not agent.me.airborne:
            agent.controller.boost = True
            if self.landing_time == -1:
                self.landing_time = agent.time
            if agent.time - self.landing_time < 0.10:
                agent.controller.handbrake = True
            else:
                agent.pop()
        if elapsed > 2:
            agent.pop()


class avoid_demo:
    def __init__(self, car):
        self.car = car
        self.direction = None
        self.start = -1

    def run(self, agent):
        botToTargetAngle = atan2(
            self.car.location[1] - agent.me.location[1],
            self.car.location[0] - agent.me.location[0],
        )
        yaw2 = atan2(agent.me.orientation[1][0], agent.me.orientation[0][0])
        if botToTargetAngle + yaw2 < 0:
            self.direction = 1
        else:
            self.direction = -1
        if self.start == -1:
            self.start = agent.time
        elapsed = agent.time - self.start
        if elapsed < 0.2:
            agent.controller.jump = True
        elif elapsed < 0.25:
            agent.controller.jump = False
            agent.controller.roll = self.direction
        elif elapsed < 0.45:
            agent.controller.jump = True
            agent.controller.roll = self.direction
        elif not agent.me.airborne:
            agent.pop()
        elif elapsed > 1:
            agent.push(recovery())


class diagonal_kickoff:
    def __init__(self, agent):
        self.step = 0
        self.side = 1 if agent.me.location[0] > 0 else 0
        self.target0 = Vector3(1788 * side(self.side), 2300 * side(agent.team), 0)
        self.target1 = Vector3(1740 * side(self.side), 2187 * side(agent.team), 0)
        self.target2 = Vector3(80 * side(self.side), 470 * side(agent.team), 0)

    def run(self, agent):
        if agent.intent is None:
            if self.step == 0:
                self.target0[0] = (agent.me.location[0] + 2 * self.target0[0]) / 3
                self.target0[1] = (agent.me.location[1] + 2 * self.target0[1]) / 3
                agent.push(goto_kickoff(self.target0))
                self.step += 1
            elif self.step == 1:
                agent.push(goto_kickoff(self.target1))
                self.step += 1
            elif self.step == 2:
                local_target1 = agent.me.local(self.target2 - agent.me.location)
                agent.push(
                    diag_flip(Vector3(1, -2 * side(agent.team) * side(self.side), 0))
                )
                self.step += 1
            elif self.step == 3:
                self.side = 1 if agent.me.location[0] > 0 else 0
                local_target2 = agent.me.local(agent.ball.location - agent.me.location)
                local_target2[0] *= 5
                agent.push(diag_flip(local_target2))


class diag_flip:
    def __init__(self, vector, cancel=False):
        self.vector = vector.normalize()
        self.pitch = abs(self.vector[0]) * -sign(self.vector[0])
        self.yaw = abs(self.vector[1]) * sign(self.vector[1])
        self.cancel = cancel
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        if elapsed < 0.09:
            agent.controller.jump = True
            agent.controller.boost = True
        elif elapsed < 0.15:
            agent.controller.jump = False
            agent.controller.yaw = self.yaw
            agent.controller.jump = False
            agent.controller.boost = True
        elif elapsed < 0.25 or (not self.cancel and elapsed < 0.9):
            if elapsed < 0.25:
                agent.controller.boost = True
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.pop()
            agent.push(recovery(Vector3(0, 0, 0)))
        # agent.controller.boost = True


class speed_flip:
    def __init__(self, direction, turn=True):
        self.direction = direction  # -1 = left, 1 = right
        self.start = -1
        self.turn = turn

    def run(self, agent):
        agent.controller.throttle = 1
        if self.start == -1:
            self.start = agent.time
            elapsed = 0
        else:
            elapsed = agent.time - self.start
            if not self.turn:
                elapsed += 0.065
        agent.controller.throttle = 1
        agent.controller.boost = True
        if elapsed < 0.065:
            agent.controller.handbrake = True
            agent.controller.steer = -1 * self.direction
        elif elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed < 0.20:
            agent.controller.jump = False
            agent.controller.pitch = -1
            agent.controller.yaw = self.direction
        elif elapsed < 0.25:
            agent.controller.jump = True
            agent.controller.pitch = -1
            agent.controller.yaw = self.direction
        elif elapsed < 0.85:
            agent.controller.pitch = 1
            agent.controller.yaw = self.direction * 0.2
            agent.controller.jump = False
        elif elapsed < 1.20:
            agent.controller.roll = self.direction
            agent.controller.handbrake = True
            agent.controller.pitch = 1
            agent.controller.yaw = self.direction * 0.70
        elif agent.me.airborne:
            agent.push(recovery())
        elif not agent.me.airborne and elapsed > 0.3:
            agent.pop()
        elif elapsed > 2:
            agent.pop()
