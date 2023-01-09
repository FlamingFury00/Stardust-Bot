# This file is for strategy

from util.objects import *
from util.routines import *
from util.tools import find_hits
from typing import Optional
from math import atan2, pi

import rlbot.utils.structures.game_data_struct
import rlbot.utils.structures.ball_prediction_struct


class BoostManagement:
    def __init__(self, agent: GoslingAgent):
        self.agent = agent
        self.big_boosts = [boost for boost in agent.boosts if boost.large]

    def get_boost_if_needed(self, max_distance: float):
        # Calcola la distanza tra il bot e la palla
        ball_distance = (self.agent.ball.location -
                         self.agent.me.location).magnitude()
        # Calcola la distanza tra il bot e l'avversario più vicino
        enemy_distance = (
            self.agent.foes[0].location - self.agent.me.location).magnitude()

        # Se la palla è più vicina a noi dell'avversario e abbiamo bisogno di boost per raggiungere la palla, prendiamo il boost più vicino
        if ball_distance < enemy_distance and self.agent.me.boost < 50 and ball_distance > 300:
            return self.get_closest_boost(max_distance)
        # Altrimenti, se l'avversario è troppo lontano da noi e abbiamo bisogno di boost per raggiungere l'avversario, prendiamo il boost più vicino
        elif enemy_distance > 500 and self.agent.me.boost < 50:
            return self.get_closest_boost(max_distance)
        # In tutti gli altri casi, non prendiamo il boost
        else:
            return None

    def get_closest_boost(self, max_distance: float):
        closest_boost = None
        closest_distance = 99999

        # Usiamo una variabile per memorizzare il boost più vicino
        # invece di scorrere la lista di boost ogni volta che viene chiamata questa funzione
        if not hasattr(self, '_cached_closest_boost'):
            self._cached_closest_boost = None
        if not hasattr(self, '_cached_closest_boost_distance'):
            self._cached_closest_boost_distance = 99999

        # Controlliamo se il boost più vicino è ancora valido e se è ancora attivo
        if self._cached_closest_boost is not None and self._cached_closest_boost.active:
            closest_boost = self._cached_closest_boost
            closest_distance = self._cached_closest_boost_distance
        # Se il boost non è più valido o non è più attivo, cerchiamo il boost più vicino
        else:
            for boost in self.big_boosts:
                if boost.active:
                    distance = (self.agent.me.location -
                                boost.location).magnitude()
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
        targets = {"opponent_goal": (
            self.agent.foe_goal.left_post, self.agent.foe_goal.right_post)}
        shots = find_hits(self.agent, targets)
        if len(shots["opponent_goal"]) > 0:
            self.agent.set_intent(shots["opponent_goal"][0])
            return
        # Altrimenti, andiamo verso la nostra porta
        self.agent.set_intent(
            goto(self.agent.friend_goal.location))
        return

    def intercept(self):
        # Calcola il punto di intercettazione tra la palla e la porta
        ball_prediction = self.agent.get_ball_prediction_struct()
        ball_location = Vector3(ball_prediction.slices[0].physics.location)
        ball_velocity = Vector3(ball_prediction.slices[0].physics.velocity)
        interception_point = self.agent.me.location + \
            self.agent.me.velocity * (
                (ball_location - self.agent.me.location).magnitude() /
                self.agent.ball.velocity.magnitude())
        # Se il punto di intercettazione è entro una certa distanza dalla nostra porta, andiamo in quella direzione
        if (interception_point - self.agent.friend_goal.location).magnitude() < 1000:
            self.agent.set_intent(goto(interception_point))
            return

        # Altrimenti, cerca di raccogliere boost se necessario
        boost = self.boost_management.get_boost_if_needed(1000)
        if boost is not None:
            self.agent.set_intent(goto(boost.location))
            return
        # Determina se siamo in grado di fare un tiro in porta
        targets = {"my_goal": (
            self.agent.foes[0].location, self.agent.foe_goal.right_post or self.agent.foe_goal.left_post)}
        shots = find_hits(self.agent, targets)
        if len(shots["my_goal"]) > 0:
            self.agent.set_intent(shots["my_goal"][0])
            return
        # Altrimenti, andiamo verso la nostra porta
        self.agent.set_intent(
            goto(self.agent.ball.location))
        return

    def execute(self):
        # Verifica se la palla si trova nella nostra metà campo
        if self.agent.ball.location.x < 0:
            # Verifica se siamo abbastanza vicini alla palla per intercettarla
            ball_distance = (self.agent.ball.location -
                             self.agent.me.location).magnitude()
            if ball_distance < 500:
                # Calcola la distanza tra il nostro bot e la nostra porta
                goal_distance = (self.agent.me.location -
                                 self.agent.friend_goal.location).magnitude()
                # Verifica se siamo più vicini alla nostra porta della palla
                if goal_distance < ball_distance:
                    # Andiamo verso la nostra porta
                    self.agent.set_intent(
                        goto(self.agent.friend_goal.location))
            else:
                # Intercettiamo la palla
                self.intercept()
        else:
            # La palla si trova nella metà campo avversaria, quindi andiamo in attacco
            self.attack()


class Bot(GoslingAgent):
    def run(self):
        self.strategy = Strategy(self)

        if self.kickoff_flag:
            self.set_intent(kickoff())

        if self.intent is not None:
            return

        self.strategy.execute()
