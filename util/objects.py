import math

import rlbot.utils.structures.game_data_struct as game_data_struct
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

# This file holds all of the objects used in gosling utils
# Includes custom vector and matrix objects


class GoslingAgent(BaseAgent):
    # This is the main object of Gosling Utils. It holds/updates information about the game and runs routines
    # All utils rely on information being structured and accessed the same way as configured in this class
    def initialize_agent(self):
        # A list of cars for both teammates and opponents
        self.friends = []
        self.foes = []
        self.role = "defender"  # Default role
        # This holds the carobject for our agent
        self.me = car_object(self.index)

        self.ball = ball_object()
        self.game = game_object()
        # A list of boosts
        self.boosts = []
        # goals
        self.friend_goal = goal_object(self.team)
        self.foe_goal = goal_object(not self.team)
        # A list that acts as the routines stack
        self.intent = None
        # Game time
        self.time = 0.0
        # Whether or not GoslingAgent has run its get_ready() function
        self.ready = False
        # the controller that is returned to the framework after every tick
        self.controller = SimpleControllerState()
        # a flag that tells us when kickoff is happening
        self.kickoff_flag = False
        # text printed at the top left of the screen
        self.debug_text = ""
        self.first_pos = Vector3(0, 0, 0)
        self.last_time = 0
        self.my_score = 0
        self.foe_score = 0

        self.rotation_index = 0

        self.LT = 0
        self.RLT = 0
        self.FTR = True
        self.ST = 0
        self.DT = 0
        self.CT = 0
        self.FPS = 120

    def get_ready(self, packet):
        # Preps all of the objects that will be updated during play
        field_info = self.get_field_info()
        for i in range(field_info.num_boosts):
            boost = field_info.boost_pads[i]
            self.boosts.append(boost_object(i, boost.location))
        self.refresh_player_lists(packet)
        self.ball.update(packet)
        self.ready = True

    def refresh_player_lists(self, packet):
        # makes new freind/foe lists
        # Useful to keep separate from get_ready because humans can join/leave a match
        self.friends = [
            car_object(i, packet)
            for i in range(packet.num_cars)
            if packet.game_cars[i].team == self.team and i != self.index
        ]
        self.foes = [
            car_object(i, packet)
            for i in range(packet.num_cars)
            if packet.game_cars[i].team != self.team
        ]

    def set_intent(self, routine):
        self.intent = routine

    def push(self, routine):
        # Shorthand for adding a routine to the stack
        # self.stack.append(routine)
        self.set_intent(routine)

    def pop(self):
        # Shorthand for clearing intent
        self.intent = None

    def line(self, start, end, color=None):
        color = color if color is not None else [255, 255, 255]
        self.renderer.draw_line_3d(start, end, self.renderer.create_color(255, *color))

    def debug_intent(self):
        # Draws the stack on the screen
        white = self.renderer.white()

        if self.intent is not None:
            text = self.intent.__class__.__name__
            self.renderer.draw_string_2d(10, 100, 3, 3, text, white)

    def clear(self):
        # Shorthand for clearing the stack of all routines
        self.stack = []

    def is_clear(self) -> bool:
        return self.intent is None

    def preprocess(self, packet):
        # Calling the update functions for all of the objects
        if packet.num_cars != len(self.friends) + len(self.foes) + 1:
            self.refresh_player_lists(packet)
        for car in self.friends:
            car.update(packet)
        for car in self.foes:
            car.update(packet)
        for pad in self.boosts:
            pad.update(packet)
        self.ball.update(packet)
        self.me.update(packet)
        self.game.update(packet)
        self.time = packet.game_info.seconds_elapsed
        # When a new kickoff begins we empty the stack
        if (
            self.kickoff_flag is False
            and packet.game_info.is_round_active
            and packet.game_info.is_kickoff_pause
        ):
            self.pop()
        # Tells us when to go for kickoff
        self.kickoff_flag = (
            packet.game_info.is_round_active and packet.game_info.is_kickoff_pause
        )

    def TimeManager(self):
        if not self.LT:
            self.LT = self.time
        else:
            if self.RLT == self.time:
                return

            if int(self.LT) != int(self.time):
                if self.FTR or self.kickoff_flag:
                    self.FTR = False
                self.ST = self.DT = 0

            Tp = round(max(1, (self.time - self.LT) * self.FPS))
            self.LT = min(self.time, self.LT + Tp)
            self.RLT = self.time
            self.CT += Tp
            if Tp > 1:
                self.ST += Tp - 1
            self.DT += 1

    def get_output(self, packet):
        # Reset controller
        self.controller.__init__()
        # Get ready, then preprocess
        if not self.ready:
            self.get_ready(packet)
        self.preprocess(packet)

        self.TimeManager()

        self.renderer.begin_rendering()
        # Run our strategy code
        self.run()
        # run the routine on the end of the stack
        if self.intent is not None:
            self.intent.run(self)
        self.renderer.end_rendering()
        # send our updated controller back to rlbot
        return self.controller

    def print_debug(self):
        white = self.renderer.white()
        self.renderer.draw_string_2d(10, 150, 3, 3, self.debug_text, white)

    def add_debug_line(self, name, vec1, vec2, color=[255, 255, 255]):
        dupes = [line for line in self.debug_lines if line.name == name]
        if len(dupes) > 0:
            return
        self.debug_lines.append(
            DebugLine(name, vec1, vec2, self.renderer.create_color(255, *color))
        )

    def remove_debug_line(self, name):
        self.debug_lines = [line for line in self.debug_lines if line.name != name]

    def clear_debug_lines(self):
        self.debug_lines = []

    def draw_debug_lines(self):
        for line in self.debug_lines:
            self.renderer.draw_line_3d(line.vec1, line.vec2, line.color)

    def is_close_to_ball(self, distance_threshold=600):
        ball_distance = (self.ball.location - self.me.location).magnitude()
        return ball_distance < distance_threshold

    def is_close_to_opponent(self, opponent, distance_threshold=600):
        opponent_distance = (opponent.location - self.me.location).magnitude()
        return opponent_distance < distance_threshold

    def get_closest_opponent(self):
        closest_opponent = self.foes[0]
        closest_distance = float("inf")

        for opponent in self.foes:
            distance = (opponent.location - self.me.location).magnitude()
            if distance < closest_distance:
                closest_distance = distance
                closest_opponent = opponent

        return closest_opponent

    def get_opponent_closest_to_ball(self):
        closest_opponent = self.foes[0]
        closest_distance = float("inf")

        for opponent in self.foes:
            distance = (opponent.location - self.ball.location).magnitude()
            if distance < closest_distance:
                closest_distance = distance
                closest_opponent = opponent

        return closest_opponent

    def get_closest_teammate(self):
        closest_teammate = self.friends[0]
        closest_distance = float("inf")

        for mate in self.friends:
            distance = (mate.location - self.me.location).magnitude()
            if distance < closest_distance:
                closest_distance = distance
                closest_teammate = mate

        return closest_teammate

    def get_best_boost(self):
        best_boost = None
        best_score = float("inf")
        available_boosts = [
            boost
            for boost in self.boosts
            if boost.large
            and boost.active
            and (boost.location - self.friend_goal.location).magnitude()
            < (self.ball.location - self.friend_goal.location).magnitude()
            and (self.foes[0].location - self.ball.location).magnitude()
            > (self.me.location - self.ball.location).magnitude()
            and self.foes[0].location.magnitude() > 1000
        ]

        for boost in available_boosts:
            bot_to_boost = (boost.location - self.me.location).normalize()
            bot_direction = self.me.forward

            distance_to_boost = (boost.location - self.me.location).magnitude()
            distance_boost_to_ball = (boost.location - self.ball.location).magnitude()
            angle = bot_direction.angle(bot_to_boost)

            # Assegna un punteggio in base alla distanza e all'angolo
            score = distance_to_boost + distance_boost_to_ball + angle * 100

            if score < best_score:
                best_score = score
                best_boost = boost

        return best_boost

    def get_closest_large_boost(self):
        available_boosts = [
            boost
            for boost in self.boosts
            if boost.large
            and boost.active
            and (boost.location - self.friend_goal.location).magnitude()
            < (self.ball.location - self.friend_goal.location).magnitude()
        ]
        closest_boost = None
        closest_distance = 10000
        for boost in available_boosts:
            distance = (self.me.location - boost.location).magnitude()
            if closest_boost is None or distance < closest_distance:
                closest_boost = boost
                closest_distance = distance
        return closest_boost

    def is_in_front_of_ball(self):
        me_to_goal = (self.me.location - self.foe_goal.location).magnitude()
        ball_to_goal = (self.ball.location - self.foe_goal.location).magnitude()
        goal_to_me = (self.me.location - self.friend_goal.location).magnitude()
        if me_to_goal > 1000 and me_to_goal < ball_to_goal and goal_to_me >= 500:
            return True
        return False

    def run(self):
        # override this with your strategy code
        pass


class car_object:
    # The carObject, and kin, convert the gametickpacket in something a little friendlier to use,
    # and are updated by GoslingAgent as the game runs
    def __init__(self, index, packet=None):
        self.location = Vector3(0, 0, 0)
        self.orientation = Matrix3(0, 0, 0)
        self.angles = Vector3(0, 0, 0)
        self.velocity = Vector3(0, 0, 0)
        self.angular_velocity = [0, 0, 0]
        self.demolished = False
        self.airborne = False
        self.supersonic = False
        self.jumped = False
        self.doublejumped = False
        self.team = 0
        self.boost = 0
        self.index = index
        if packet is not None:
            self.team = packet.game_cars[self.index].team
            self.update(packet)

    def local(self, value):
        # Shorthand for self.matrix.dot(value)
        return self.orientation.dot(value)

    def update(self, packet):
        car = packet.game_cars[self.index]
        self.location.data = [
            car.physics.location.x,
            car.physics.location.y,
            car.physics.location.z,
        ]
        self.velocity.data = [
            car.physics.velocity.x,
            car.physics.velocity.y,
            car.physics.velocity.z,
        ]
        self.angles = Vector3(
            car.physics.rotation.pitch,
            car.physics.rotation.yaw,
            car.physics.rotation.roll,
        )
        self.orientation = Matrix3(
            car.physics.rotation.pitch,
            car.physics.rotation.yaw,
            car.physics.rotation.roll,
        )
        self.angular_velocity = self.orientation.dot(
            [
                car.physics.angular_velocity.x,
                car.physics.angular_velocity.y,
                car.physics.angular_velocity.z,
            ]
        ).data
        self.demolished = car.is_demolished
        self.airborne = not car.has_wheel_contact
        self.supersonic = car.is_super_sonic
        self.jumped = car.jumped
        self.doublejumped = car.double_jumped
        self.boost = car.boost

    @property
    def forward(self):
        # A vector pointing forwards relative to the cars orientation. Its magnitude is 1
        return self.orientation.forward

    @property
    def left(self):
        # A vector pointing left relative to the cars orientation. Its magnitude is 1
        return self.orientation.left

    @property
    def up(self):
        # A vector pointing up relative to the cars orientation. Its magnitude is 1
        return self.orientation.up


class ball_object:
    def __init__(self):
        self.location = Vector3(0, 0, 0)
        self.velocity = Vector3(0, 0, 0)
        self.latest_touched_time = 0
        self.latest_touched_team = 0

    def update(self, packet):
        ball = packet.game_ball
        self.location.data = [
            ball.physics.location.x,
            ball.physics.location.y,
            ball.physics.location.z,
        ]
        self.velocity.data = [
            ball.physics.velocity.x,
            ball.physics.velocity.y,
            ball.physics.velocity.z,
        ]
        self.latest_touched_time = ball.latest_touch.time_seconds
        self.latest_touched_team = ball.latest_touch.team


class boost_object:
    def __init__(self, index, location):
        self.index = index
        self.location = Vector3(location.x, location.y, location.z)
        self.active = True
        self.large = self.location.z > 7

    def update(self, packet):
        self.active = packet.game_boosts[self.index].is_active


class goal_object:
    # This is a simple object that creates/holds goalpost locations for a given team (for soccer on standard maps only)
    def __init__(self, team):
        team = 1 if team == 1 else -1
        self.location = Vector3(0, team * 5100, 320)  # center of goal line
        # Posts are closer to x=750, but this allows the bot to be a little more accurate
        self.left_post = Vector3(team * 850, team * 5100, 320)
        self.right_post = Vector3(-team * 850, team * 5100, 320)


class game_object:
    # This object holds information about the current match
    def __init__(self):
        self.time = 0
        self.time_remaining = 0
        self.overtime = False
        self.round_active = False
        self.kickoff = False
        self.match_ended = False

    def update(self, packet):
        game = packet.game_info
        self.time = game.seconds_elapsed
        self.time_remaining = game.game_time_remaining
        self.overtime = game.is_overtime
        self.round_active = game.is_round_active
        self.kickoff = game.is_kickoff_pause
        self.match_ended = game.is_match_ended


class Matrix3:
    # The Matrix3's sole purpose is to convert roll, pitch, and yaw data from the gametickpaket into an orientation matrix
    # An orientation matrix contains 3 Vector3's
    # Matrix3[0] is the "forward" direction of a given car
    # Matrix3[1] is the "left" direction of a given car
    # Matrix3[2] is the "up" direction of a given car
    # If you have a distance between the car and some object, ie ball.location - car.location,
    # you can convert that to local coordinates by dotting it with this matrix
    # ie: local_ball_location = Matrix3.dot(ball.location - car.location)
    def __init__(self, pitch, yaw, roll):
        CP = math.cos(pitch)
        SP = math.sin(pitch)
        CY = math.cos(yaw)
        SY = math.sin(yaw)
        CR = math.cos(roll)
        SR = math.sin(roll)
        # List of 3 vectors, each descriping the direction of an axis: Forward, Left, and Up
        self.data = [
            Vector3(CP * CY, CP * SY, SP),
            Vector3(CY * SP * SR - CR * SY, SY * SP * SR + CR * CY, -CP * SR),
            Vector3(-CR * CY * SP - SR * SY, -CR * SY * SP + SR * CY, CP * CR),
        ]
        self.forward, self.left, self.up = self.data

    def __getitem__(self, key):
        return self.data[key]

    def dot(self, vector):
        return Vector3(
            self.forward.dot(vector), self.left.dot(vector), self.up.dot(vector)
        )


class Vector3:
    # This is the backbone of Gosling Utils.
    # The Vector3 makes it easy to store positions, velocities, etc and perform vector math
    # A Vector3 can be created with:
    # - Anything that has a __getitem__ (lists, tuples, Vector3's, etc)
    # - 3 numbers
    # - A gametickpacket vector
    def __init__(self, *args):
        if hasattr(args[0], "__getitem__"):
            self.data = list(args[0])
        elif isinstance(args[0], game_data_struct.Vector3):
            self.data = [args[0].x, args[0].y, args[0].z]
        elif isinstance(args[0], game_data_struct.Rotator):
            self.data = [args[0].pitch, args[0].yaw, args[0].roll]
        elif len(args) == 3:
            self.data = list(args)
        else:
            raise TypeError("Vector3 unable to accept %s" % args)

    # Property functions allow you to use `Vector3.x` vs `Vector3[0]`
    @property
    def x(self):
        return self.data[0]

    @x.setter
    def x(self, value):
        self.data[0] = value

    @property
    def y(self):
        return self.data[1]

    @y.setter
    def y(self, value):
        self.data[1] = value

    @property
    def z(self):
        return self.data[2]

    @z.setter
    def z(self, value):
        self.data[2] = value

    def __getitem__(self, key):
        # To access a single value in a Vector3, treat it like a list
        # ie: to get the first (x) value use: Vector3[0]
        # The same works for setting values
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    def __str__(self):
        # Vector3's can be printed to console
        return str(self.data)

    __repr__ = __str__

    def __eq__(self, value):
        # Vector3's can be compared with:
        # - Another Vector3, in which case True will be returned if they have the same values
        # - A single value, in which case True will be returned if the Vector's length matches the value
        if hasattr(value, "__getitem__"):
            return self.data == value.data
        else:
            return self.magnitude() == value

    # Vector3's support most operators (+-*/)
    # If using an operator with another Vector3, each dimension will be independent
    # ie x+x, y+y, z+z
    # If using an operator with only a value, each dimension will be affected by that value
    # ie x+v, y+v, z+v
    def __add__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] + value[0], self[1] + value[1], self[2] + value[2])
        return Vector3(self[0] + value, self[1] + value, self[2] + value)

    __radd__ = __add__

    def __sub__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] - value[0], self[1] - value[1], self[2] - value[2])
        return Vector3(self[0] - value, self[1] - value, self[2] - value)

    __rsub__ = __sub__

    def __neg__(self):
        return Vector3(-self[0], -self[1], -self[2])

    def __mul__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] * value[0], self[1] * value[1], self[2] * value[2])
        return Vector3(self[0] * value, self[1] * value, self[2] * value)

    __rmul__ = __mul__

    def __truediv__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] / value[0], self[1] / value[1], self[2] / value[2])
        return Vector3(self[0] / value, self[1] / value, self[2] / value)

    def __rtruediv__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(value[0] / self[0], value[1] / self[1], value[2] / self[2])
        raise TypeError("unsupported rtruediv operands")

    def __abs__(self):
        return Vector3(abs(self[0]), abs(self[1]), abs(self[2]))

    def magnitude(self):
        # Magnitude() returns the length of the vector
        return math.sqrt(
            (self[0] * self[0]) + (self[1] * self[1]) + (self[2] * self[2])
        )

    def normalize(self):
        # Normalize() returns a Vector3 that shares the same direction but has a length of 1.0
        magnitude = self.magnitude()
        if magnitude != 0:
            return Vector3(
                self[0] / magnitude, self[1] / magnitude, self[2] / magnitude
            )
        return Vector3(0, 0, 0)

    # Linear algebra functions
    def dot(self, value):
        return self[0] * value[0] + self[1] * value[1] + self[2] * value[2]

    def cross(self, value):
        # A .cross((0, 0, 1)) will rotate the vector counterclockwise by 90 degrees
        return Vector3(
            (self[1] * value[2]) - (self[2] * value[1]),
            (self[2] * value[0]) - (self[0] * value[2]),
            (self[0] * value[1]) - (self[1] * value[0]),
        )

    def distance(self, value):
        # returns the distance between vectors
        return (value - self).magnitude()

    def flatten(self):
        # Sets Z (Vector3[2]) to 0
        return Vector3(self[0], self[1], 0)

    def render(self):
        # Returns a list with the x and y values, to be used with pygame
        return [self[0], self[1]]

    def copy(self):
        # Returns a copy of this Vector3
        return Vector3(self.data[:])

    def angle(self, value):
        # Returns the angle between this Vector3 and another Vector3
        return math.acos(
            round(self.flatten().normalize().dot(value.flatten().normalize()), 4)
        )

    def angle3D(self, value) -> float:
        # Returns the angle between this Vector3 and another Vector3
        return math.acos(round(self.normalize().dot(value.normalize()), 4))

    def rotate(self, angle):
        # Rotates this Vector3 by the given angle in radians
        # Note that this is only 2D, in the x and y axis
        return Vector3(
            (math.cos(angle) * self[0]) - (math.sin(angle) * self[1]),
            (math.sin(angle) * self[0]) + (math.cos(angle) * self[1]),
            self[2],
        )

    def clamp(self, start, end):
        # Similar to integer clamping, Vector3's clamp() forces the Vector3's direction between a start and end Vector3
        # Such that Start < Vector3 < End in terms of clockwise rotation
        # Note that this is only 2D, in the x and y axis
        s = self.normalize()
        right = s.dot(end.cross((0, 0, -1))) < 0
        left = s.dot(start.cross((0, 0, -1))) > 0
        if (
            (right and left)
            if end.dot(start.cross((0, 0, -1))) > 0
            else (right or left)
        ):
            return self
        if start.dot(s) < end.dot(s):
            return end
        return start


class DebugLine:
    def __init__(self, name, vec1, vec2, color) -> None:
        self.name = name
        self.vec1 = vec1
        self.vec2 = vec2
        self.color = color
