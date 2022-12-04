# This file is for strategy

from util.objects import *
from util.routines import *
from util.tools import find_hits


class Bot(GoslingAgent):
    # This function runs every in-game tick (every time the game updates anything)
    def run(self):
        # the line below tells the bot what it's trying to do

        d1 = abs(self.me.location.y - self.foe_goal.location.y)
        d2 = abs(self.ball.location.y - self.foe_goal.location.y)
        is_in_front_of_ball = d2 > d1

        if len(self.stack) != 0:
            return

        if self.kickoff_flag:
            self.set_intent(kickoff())
            return

        # if is_in_front_of_ball:
        #     self.set_intent(goto(self.friend_goal.location))
        #     return

        big_boost_pads = [boost for boost in self.boosts if (
            boost.large and boost.active)]

        closest_boost = None
        closest_distance = 99999

        # small_boost_pads = [boost for boost in self.boosts if boost.active]

        targets = {
            "opponent_goal": (self.foe_goal.left_post, self.foe_goal.right_post),
            "not_my_net": (self.friend_goal.right_post, self.friend_goal.left_post)
        }

        shots = find_hits(self, targets)

        if len(shots["opponent_goal"]) > 0 and self.me.boost >= 90:
            self.push(short_shot(self.foe_goal.location))
            print("attack")
            return

        if self.me.boost <= 10 and len(self.stack) == 0:
            for boost in big_boost_pads:
                distance = (self.me.location - boost.location).magnitude()
                if closest_boost is None or distance < closest_distance:
                    closest_boost = boost
                    closest_distance = distance

            if closest_boost is not None:
                self.push(goto(closest_boost.location))
                print("i'll take boost")
                return

        # if len(self.stack) == 0 and not is_in_front_of_ball:
        #     self.set_intent(goto(self.friend_goal.location))
        #     return

        if len(shots["not_my_net"]) > 0:
            self.push(shots["not_my_net"][0])
            print("defend")
            return
