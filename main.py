# This file is for strategy

from util.objects import *
from util.routines import *
from util.tools import find_hits


class BoostManager:
    def __init__(self, agent):
        self.agent = agent
        self.big_boosts = [boost for boost in agent.boosts if boost.large]
        self.small_boosts = [
            boost for boost in agent.boosts if not boost.large]

    def get_boost_if_needed(self):
        if self.agent.me.boost < 50:
            return self.get_closest_boost()

    def get_closest_boost(self):
        if len(self.big_boosts) > 0:
            return self.get_closest_boost_from_list(self.big_boosts)
        elif len(self.small_boosts) > 0:
            return self.get_closest_boost_from_list(self.small_boosts)

    def get_closest_boost_from_list(self, boost_list):
        closest_boost = None
        closest_distance = 99999

        for boost in boost_list:
            if boost.active:
                distance = (self.agent.me.location -
                            boost.location).magnitude()
                if closest_boost is None or distance < closest_distance:
                    closest_boost = boost
                    closest_distance = distance
        return closest_boost


class Bot(GoslingAgent):
    # This function runs every in-game tick (every time the game updates anything)
    def run(self):
        self.boost_manager = BoostManager(self)
        # the line below tells the bot what it's trying to do

        d1 = abs(self.me.location.y - self.foe_goal.location.y)
        d2 = abs(self.ball.location.y - self.foe_goal.location.y)
        is_in_front_of_ball = d2 > d1

        if self.intent is not None:
            return

        if self.kickoff_flag:
            self.set_intent(kickoff())
            return

        # if is_in_front_of_ball:
        #     self.set_intent(goto(self.friend_goal.location))
        #     return

        if self.me.boost < 50:
            boost = self.boost_manager.get_boost_if_needed()
            if boost is not None:
                print("i'll take boost")
                self.set_intent(goto(boost.location))
                return

        ball_distance = (self.ball.location - self.me.location).magnitude()
        enemy_distance = (self.foes[0].location - self.me.location).magnitude()

        # Se la palla è più vicina a noi dell'avversario, attacchiamo
        if ball_distance < enemy_distance:
            # Determina se siamo in grado di fare un tiro in porta
            targets = {"opponent_goal": (
                self.foe_goal.left_post, self.foe_goal.right_post)}
            shots = find_hits(self, targets)
            if len(shots["opponent_goal"]) > 0:
                self.set_intent(shots["opponent_goal"][0])
                return
            # Altrimenti, andiamo verso la palla
            self.set_intent(goto(self.ball.location))
        # Se l'avversario è più vicino della palla, difendiamo
        else:
            # Determina se siamo in grado di fare un tiro verso la nostra porta
            targets = {"not_my_net": (
                self.friend_goal.right_post, self.friend_goal.left_post)}
            shots = find_hits(self, targets)
            if len(shots["not_my_net"]) > 0:
                self.set_intent(shots["not_my_net"][0])
                return
            # Altrimenti, andiamo verso l'avversario
            self.set_intent(goto(self.foes[0].location))
