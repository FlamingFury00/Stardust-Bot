# This file is for strategy

from util.objects import *
from util.routines import *
from util.tools import find_hits
from typing import Optional
from math import atan2, pi


class BoostManagement:
    def __init__(self, agent: GoslingAgent):

        BOOSTS = [Vector3(3072.0, side(agent.team) * 4096.0, 73.0),
                  Vector3(- 3072.0, side(agent.team) * 4096.0, 73.0),
                  Vector3(side(agent.team) * 3584.0, 0.0, 73.0)]

        self.agent = agent
        self.big_boosts = [boost for boost in BOOSTS]

    def get_boost_if_needed(self, max_distance: float):
        ball_location = self.agent.ball.location
        distance_to_ball = (ball_location - self.agent.me.location).magnitude()
        # Calcola la distanza tra la palla e ogni avversario
        enemy_distances = [(ball_location - enemy.location).magnitude()
                           for enemy in self.agent.foes]
        # controlla se c'è un avversario più vicino della palla
        if min(enemy_distances) < distance_to_ball and self.agent.me.boost < 50 and distance_to_ball > 1500:
            return self.get_closest_boost(max_distance)
        else:
            return None

    def get_closest_boost(self, max_distance: float):
        closest_boost = None
        closest_distance = 10000

        # Usiamo una variabile per memorizzare il boost più vicino
        # invece di scorrere la lista di boost ogni volta che viene chiamata questa funzione
        if not hasattr(self, '_cached_closest_boost'):
            self._cached_closest_boost = None
        if not hasattr(self, '_cached_closest_boost_distance'):
            self._cached_closest_boost_distance = 10000

        # Controlliamo se il boost più vicino è ancora valido e se è ancora attivo
        if self._cached_closest_boost is not None:
            closest_boost = self._cached_closest_boost
            closest_distance = self._cached_closest_boost_distance
        # Se il boost non è più valido o non è più attivo, cerchiamo il boost più vicino
        else:
            for boost in self.big_boosts:
                # if boost.active:
                distance = (self.agent.me.location -
                            boost).magnitude()
                if distance < max_distance and (closest_boost is None or distance < closest_distance):
                    closest_boost = boost
                    closest_distance = distance
            if closest_boost is not None:
                self._cached_closest_boost = closest_boost
                self._cached_closest_boost_distance = closest_distance
                return closest_boost


class Strategy:
    def __init__(self, agent: GoslingAgent):
        self.agent = agent
        self.boost_management = BoostManagement(agent)

    def attack(self):
        # Determina se siamo in grado di fare un tiro in porta
        targets = {"goal": (self.agent.foe_goal.left_post,
                            self.agent.foe_goal.right_post)}
        shots = find_hits(self.agent, targets)
        if len(shots["goal"]) > 0:
            print("i'll attack")
            return self.agent.set_intent(shots["goal"][0])
        # elif (self.agent.me.location - self.agent.ball.location).magnitude() < (self.agent.foes[0].location - self.agent.ball.location).magnitude() and self.agent.me.boost >= 50:
        #     self.agent.set_intent(short_shot(self.agent.foe_goal.location))
        #     return
        else:
            # Altrimenti, andiamo verso la nostra porta
            self.agent.set_intent(
                goto(self.agent.friend_goal.location - Vector3(0, side(self.agent.team) * 800, 0), self.agent.friend_goal.location))

    def intercept(self):
        # Altrimenti, portiamo la palla verso la nostra porta
        midleft = Vector3(0, cap(self.agent.ball.location.y -
                          side(self.agent.team) * 2000, 4000, -4000), 500)
        midright = Vector3(0, cap(self.agent.ball.location.y -
                           side(self.agent.team) * 2000, 4000, -4000), 500)
        upfield_left = Vector3(-side(self.agent.team) * 2500,
                               self.agent.ball.location.y - side(self.agent.team) * 2000, 500)
        upfield_right = Vector3(side(self.agent.team) * 2500,
                                self.agent.ball.location.y - side(self.agent.team) * 2000, 500)
        targets = {"my_goal": (self.agent.friend_goal.right_post, self.agent.friend_goal.left_post), "left": (
            upfield_left, midleft), "right": (midright, upfield_right)}
        shots = find_hits(self.agent, targets)
        if len(shots["my_goal"]) > 0:
            return self.agent.set_intent(shots["my_goal"][0])
        elif len(shots["right"]) > 0:
            return self.agent.set_intent(shots["right"][0])
        elif len(shots["left"]) > 0:
            return self.agent.set_intent(shots["left"][0])
        # else:
        #     self.agent.set_intent(short_shot(self.agent.foe_goal.location))
        #     return

    def execute(self):
        print(self.agent.ball.location.y * side(self.agent.team))
        if (self.agent.foe_goal.location - self.agent.ball.location).magnitude() > 1500:
            boost = self.boost_management.get_boost_if_needed(3000)
            if boost is not None:
                return self.agent.set_intent(goto(boost))
        # Verifica se la palla si trova nella nostra metà campo
        if self.agent.ball.location.y * side(self.agent.team) > 0:
            # Verifica se siamo abbastanza vicini alla palla per intercettarla
            ball_distance = (self.agent.ball.location -
                             self.agent.me.location).magnitude()
            # if ball_distance < 500:
            # Calcola la distanza tra il nostro bot e la nostra porta
            goal_distance = (self.agent.me.location -
                             self.agent.friend_goal.location).magnitude()
            # Verifica se siamo più vicini alla nostra porta della palla
            if goal_distance < ball_distance:
                # Intercettiamo la palla
                self.intercept()
            elif self.agent.me.boost >= 50 and (goal_distance < ball_distance or (self.agent.foes[0].location - self.agent.ball.location).magnitude() < 500 or (self.agent.friend_goal.location - self.agent.ball.location).magnitude() < 1000):
                return self.agent.set_intent(short_shot(
                    self.agent.foe_goal.location))
            else:
                self.attack()
            # else:
            #     # Attachiamo
            #     self.attack()
        elif (self.agent.foes[0].location - self.agent.ball.location).magnitude() > 500:
            # Prima, cerca di raccogliere boost se necessario
            boost = self.boost_management.get_boost_if_needed(1500)
            if boost is not None:
                self.agent.set_intent(goto(boost))
                return
            # La palla si trova nella metà campo avversaria, quindi andiamo in attacco
            self.attack()
        else:
            self.agent.set_intent(
                goto(self.agent.friend_goal.location - Vector3(0, side(self.agent.team) * 800, 0)))


class Bot(GoslingAgent):
    def run(self):
        self.strategy = Strategy(self)

        if self.intent is not None:
            return

        if self.kickoff_flag:
            self.set_intent(kickoff())
            return

        # if self.me.location.x >= 4096:
        #     self.set_intent(flip(self.ball.location))

        self.strategy.execute()
