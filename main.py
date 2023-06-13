from math import acos, pi

from util.objects import *
from util.routines import *
from util.tools import *


class Bot(GoslingAgent):
    def should_rotate(self):
        if is_last_one_back(self) and not in_goal_area(self):
            return False

        if is_ball_centering(self) and not is_ahead_of_ball(self):
            return False

        if demo_rotation(self) and not is_ahead_of_ball(self):
            return False

        if friends_ahead_of_ball(self) == 0 and not is_ahead_of_ball(self):
            return False

        return True

    def run(self):
        ball_to_me = self.me.location - self.ball.location

        if self.intent is not None:
            return

        # Cooperation
        if len(self.friends) > 0:
            if self.kickoff_flag and is_closest_kickoff(self, self.me):
                self.set_intent(kickoff(self.me.location.x))
                return

            if self.kickoff_flag and is_second_closest_kickof(self):
                return

            # if is_ball_going_towards_goal(self) and is_closest(self, self.me):
            #     self.set_intent(short_shot(self.foe_goal.location))
            #     return

            # Rotation and positioning
            if self.should_rotate():
                desired_zone = zone_5_positioning(self)
                if desired_zone is not None:
                    return self.set_intent(goto(desired_zone[0]))

            if self.is_in_front_of_ball() and are_no_bots_back(self):
                return self.set_intent(goto(self.friend_goal.location))

            # Boost grabbing
            if self.me.boost < 30:
                target_boost = self.get_best_boost()
                if target_boost is not None:
                    self.set_intent(steal_boost(target_boost))
                    return

            # Defence
            # if is_ball_going_towards_goal(self) and is_second_closest(self):
            best_save = find_best_save(self, self.get_closest_opponent())
            if best_save is not None:
                self.set_intent(best_save)
                return

            # Attack
            if friends_attacking(self) == 0:
                best_shot = find_best_shot(self, self.get_closest_opponent())
                if best_shot is not None:
                    self.set_intent(best_shot)
                    return

            # Team play
            if friendly_cars_in_front_of_goal(self) and is_ball_centering(self):
                self.set_intent(center_ball(self.foe_goal.location))
                return

            # Dribbling
            if self.me.boost > 30 and self.is_close_to_ball(200):
                self.set_intent(dribble(self.foe_goal.location))
                return

            # Demo and avoid demo
            if self.me.boost > 50:
                closest_opponent = self.get_closest_opponent()
                if closest_opponent is not None and self.is_close_to_opponent(
                    closest_opponent
                ):
                    self.set_intent(demo(closest_opponent))
                    return
                # else:
                #     demo_intent = detect_demo(self)
                #     if demo_intent is not None:
                #         self.set_intent(avoid_demo(self.me))

            if are_no_bots_back(self) and is_second_closest(self):
                return self.set_intent(goto(self.friend_goal.location))
        # Alone
        else:
            if self.kickoff_flag:
                self.set_intent(kickoff(self.me.location.x))
                return

            # Boost grabbing
            if self.me.boost < 30 and is_closest(self, self.me):
                target_boost = self.get_closest_large_boost()
                if target_boost is not None:
                    self.set_intent(steal_boost(target_boost))
                    return

            if self.is_in_front_of_ball() and self.intent is None:
                return self.set_intent(goto(self.ball.location, self.foes[0].location))

            # if (
            #     self.friend_goal.location - self.me.location
            # ).magnitude() < 1000 and self.ball.location.magnitude() < 700:
            #     self.set_intent(align_in_goal())
            #     return

            # Attack
            best_shot = find_best_shot(self, self.get_closest_opponent())
            if best_shot is not None:
                self.set_intent(best_shot)
                return

            # Dribbling
            if self.me.boost > 30 and self.is_close_to_ball(200):
                self.set_intent(dribble(self.foe_goal.location))
                return

            # Defence
            # if is_ball_going_towards_goal(self):
            # best_save = find_best_save(self, self.get_closest_opponent())
            # if best_save is not None:
            #     self.set_intent(best_save)
            #     return

        # If none of the previous conditions are met, the bot positions itself in our home
        # self.set_intent(goto(self.friend_goal.location))
