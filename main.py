# This file is for strategy

from util.objects import *
from util.routines import *
from util.tools import find_hits
from typing import Optional

import rlbot.utils.structures.game_data_struct as game_data
import rlbot.utils.structures.ball_prediction_struct as ball_prediction


class Strategy:
    def __init__(self, agent: GoslingAgent):
        self.agent = agent
        self.big_boosts = [boost for boost in agent.boosts if boost.large]
        self.small_boosts = [
            boost for boost in agent.boosts if not boost.large]

    def get_boost_if_needed(self):
        # Calcola la distanza tra il bot e la palla
        ball_distance = (self.agent.ball.location -
                         self.agent.me.location).magnitude()
        # Calcola la distanza tra il bot e l'avversario più vicino
        enemy_distance = (
            self.agent.foes[0].location - self.agent.me.location).magnitude()
        # Se la palla è più vicina a noi dell'avversario, prendiamo il boost solo se ne abbiamo bisogno
        if ball_distance < enemy_distance:
            if self.agent.me.boost < 50:
                return self.get_closest_boost()
        # Altrimenti, prendiamo il boost solo se ne abbiamo bisogno e se l'avversario è troppo lontano da noi
        elif enemy_distance > 1000 and self.agent.me.boost < 50:
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

    def attack(self):
        # Determina se siamo in grado di fare un tiro in porta
        targets = {"opponent_goal": (
            self.agent.foe_goal.left_post, self.agent.foe_goal.right_post)}
        shots = find_hits(self.agent, targets)
        if len(shots["opponent_goal"]) > 0:
            self.agent.set_intent(shots["opponent_goal"][0])
            return
        # Altrimenti, andiamo verso la palla
        self.agent.set_intent(goto(self.agent.ball.location))

    def defend(self):
        # Ottieni la previsione della palla
        prediction = self.agent.get_ball_prediction_struct()
        if prediction is not None:
            # Cerca la previsione della palla più vicina all'istante corrente
            closest_prediction = prediction.slices[0]
            closest_distance = abs(
                closest_prediction.game_seconds - self.agent.time)
            for i in range(1, prediction.num_slices):
                slice = prediction.slices[i]
                distance = abs(slice.game_seconds - self.agent.time)
                if distance < closest_distance:
                    closest_prediction = slice
                    closest_distance = distance
            # Calcola la posizione futura dell'auto utilizzando la previsione della palla più vicina all'istante corrente
            car_location = self.agent.me.location + \
                self.agent.me.velocity * closest_prediction.game_seconds
            # Calcola la distanza tra l'auto e la porta da difendere
            distance_to_goal = (
                car_location - self.agent.friend_goal.location).magnitude()
            # Se l'auto è già vicina alla porta, rimani lì
            if distance_to_goal < 500:
                boost = self.get_closest_boost()
                if boost is not None:
                    return goto(boost.location)
            # Altrimenti, vai verso la porta
            else:
                return goto(self.agent.friend_goal.location)
        # Se non abbiamo una previsione della palla, vai verso la palla
        return goto(self.agent.ball.location)


class Bot(GoslingAgent):
    def run(self):
        self.strategy = Strategy(self)

        if self.kickoff_flag:
            self.set_intent(kickoff())
            return

        if self.intent is not None:
            return

        # Utilizza la funzione get_ball_prediction_struct di Rlbot per prevedere dove andrà la palla
        prediction = self.get_ball_prediction_struct()
        if prediction is not None:
            # Calcola la distanza tra il bot e la palla
            ball_distance = (self.ball.location - self.me.location).magnitude()
            # Calcola la distanza tra il bot e l'avversario più vicino
            enemy_distance = (
                self.foes[0].location - self.me.location).magnitude()

            # Se la palla è più vicina a noi dell'avversario oppure l'avversario è più lontano dalla sua porta di quanto non lo sia la palla, attacchiamo
            if ball_distance < enemy_distance or (self.foes[0].location - self.foe_goal.location).magnitude() > (self.ball.location - self.foe_goal.location).magnitude():
                self.strategy.attack()
            # Se l'avversario è più vicino della palla, difendiamo
            else:
                self.set_intent(self.strategy.defend())
        else:
            boost = self.strategy.get_boost_if_needed()
            if boost is not None:
                self.set_intent(goto(boost.location))
                return
