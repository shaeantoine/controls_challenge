from . import BaseController
import pickle
import pandas as pd

class Controller(BaseController):
    def __init__(self): 
        self.alpha = 0.1
        self.beta = 0.001
        self.gamma = 0.001

        # Load the model
        with open("l_reg.pkl", "rb") as f:
            self.loaded_model = pickle.load(f)

    def predict(self, current, future, roll, vel, accel):

        X = pd.DataFrame([[current, future, roll, accel, vel]], 
                    columns=['current_lataccel', 'targetLateralAcceleration', 'roll', 'aEgo', 'vEgo'])  

        steer_prediction = self.loaded_model.predict(X)[0]

        return steer_prediction

    def update(self, target_lataccel, current_lataccel, state, future_plan):
        # Basic error
        error = target_lataccel - current_lataccel

        # Decomposing state - named tuple
        state_roll = state.roll_lataccel
        state_v = state.v_ego
        state_a = state.a_ego

        # Decomposing future_plan - named tuple
        future_lataccel = future_plan.lataccel
        future_roll = future_plan.roll_lataccel
        future_v = future_plan.v_ego
        future_a = future_plan.a_ego

        # Calculate steer
        steer = self.predict(current_lataccel, target_lataccel, state_roll, state_v, state_a)

        return steer