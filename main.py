from util.common import (
    are_no_bots_back,
    demo_rotation,
    detect_demo,
    friendly_cars_in_front_of_goal,
    friends_ahead_of_ball,
    friends_attacking,
    in_goal_area,
    is_ahead_of_ball,
    is_ball_centering,
    is_closest,
    is_closest_kickoff,
    is_last_one_back,
    is_second_closest,
    is_second_closest_kickof,
    should_attack,
    should_defend,
    zone_5_positioning,
)
from util.objects import GoslingAgent
from util.routines import (
    avoid_demo,
    center_ball,
    demo,
    dribble,
    go_centre,
    goto,
    kickoff,
    kickoff2,
    steal_boost,
)
from util.tools import find_best_save, find_best_shot


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
                return self.set_intent(
                    goto(self.friend_goal.location, self.ball.velocity)
                )

            # Boost grabbing
            if self.me.boost < 30 and is_second_closest(self):
                target_boost = self.get_best_boost()
                if target_boost is not None:
                    self.set_intent(steal_boost(target_boost))
                    return

            # if (
            #     should_attack(self)
            #     and friends_attacking(self) >= 1
            #     and self.get_closest_teammate().location.distance(self.ball.location)
            #     < self.me.location.distance(self.ball.location)
            #     and not friends_ahead_of_ball(self)
            # ):
            #     return self.set_intent(go_centre())

            if (
                should_attack(self)
                and not friends_ahead_of_ball(self)
                and friends_attacking(self) >= 1
                and (self.ball.latest_touched_team != self.me.team)
                and self.me.boost >= 30
            ):
                self.set_intent(demo(self.get_opponent_closest_to_ball()))
                return

            demo_intent = detect_demo(self)
            if demo_intent[1] is not None:
                self.set_intent(avoid_demo(self.get_closest_opponent()))
                return

            if (
                should_attack(self)
                and not friends_ahead_of_ball(self)
                and friends_attacking(self) >= 1
                and (self.ball.latest_touched_team != self.me.team)
            ):
                self.set_intent(go_centre())
                return

            # Attack
            if should_attack(self):
                best_shot = find_best_shot(self, self.get_closest_opponent())
                if best_shot is not None:
                    self.set_intent(best_shot)
                    return

            # Dribbling
            if self.me.boost > 30 and self.is_close_to_ball(200):
                self.set_intent(dribble(self.foe_goal.location))
                return

            # Team play
            if friendly_cars_in_front_of_goal(self) and is_ball_centering(self):
                self.set_intent(center_ball(self.foe_goal.location))
                return

            # Defence
            # if is_ball_going_towards_goal(self) and is_second_closest(self):
            best_save = find_best_save(self, self.get_closest_opponent())
            if best_save is not None:
                self.set_intent(best_save)
                return

            # Demo and avoid demo
            # if self.me.boost > 50:
            #     closest_opponent = self.get_closest_opponent()
            #     if closest_opponent is not None and self.is_close_to_opponent(
            #         closest_opponent
            #     ):
            #         self.set_intent(demo(closest_opponent))
            #         return
            # else:
            #     demo_intent = detect_demo(self)
            #     if demo_intent[1] is not None and not demo_intent[0]:
            #         self.set_intent(avoid_demo(self.get_closest_opponent()))

            if are_no_bots_back(self) and is_second_closest(self):
                return self.set_intent(goto(self.friend_goal.location))
        # Alone
        else:
            if self.kickoff_flag:
                self.set_intent(kickoff2())
                return

            # Rotation and positioning
            if self.should_rotate():
                desired_zone = zone_5_positioning(self)
                if desired_zone is not None:
                    return self.set_intent(goto(desired_zone[0]))

            if self.is_in_front_of_ball():
                self.set_intent(goto(self.friend_goal.location))

            # Boost grabbing
            if (
                self.me.boost < 30
                and is_closest(self, self.me)
                # and self.get_closest_opponent().location.magnitude() > 1000
            ):
                target_boost = self.get_closest_large_boost()
                if target_boost is not None:
                    self.set_intent(steal_boost(target_boost))
                    return

            # if (
            #     self.friend_goal.location - self.me.location
            # ).magnitude() < 1000 and self.ball.location.magnitude() < 700:
            #     self.set_intent(align_in_goal())
            #     return

            # Dribbling
            # if self.me.boost > 30 and self.is_close_to_ball(200):
            #     self.set_intent(dribble(self.foe_goal.location))
            #     return

            # Attack
            if should_attack(self):
                best_shot = find_best_shot(self, self.get_closest_opponent())
                if best_shot is not None:
                    self.set_intent(best_shot)
                    return

            # Defence
            # if is_ball_going_towards_goal(self):
            if should_defend(self):
                best_save = find_best_save(self, self.get_closest_opponent())
                if best_save is not None:
                    self.set_intent(best_save)
                    return

        # If none of the previous conditions are met, the bot positions itself in our home
        # self.set_intent(goto(self.friend_goal.location))
