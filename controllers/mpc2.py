from . import BaseController
import numpy as np
from random import uniform

class controller(BaseController):
   def __init__(self):
        self.penis = "penis"

   '''
   The system dynamics of the car will likely be varied
   since the data is from a bunch of different cars in 
   a bunch of different conditions. SOURCE OF ERRORS
   '''
   def dynamics(lataccel, steering):
      return lataccel + steering
    

   '''
   This function is responsible for generating the 
   next 10 steps of current lateral acceleration,
   based on the system dynamics model 
   '''
   def predict(self, current_lataccel):
      # We need to generate some steering angle for the next 10 steps
      steer_command = [(uniform(-1, 1)) for i in range(10)] 
      future_lataccel = np.zeros(10)

      '''
      Generate future lataccel by adding the steering commands off of 
      the last steer command.
      '''
      for i in range(steer_command):
         # First prediction 
         if i == 0:
            future_lataccel[i] = current_lataccel + steer_command[i]
         else: # The rest
            future_lataccel[i] = self.dynamics(future_lataccel[i-1], steer_command[i-1])

      return future_lataccel


   '''
   Here we will evaluate the cost of the predictions 
   vs the target lateral acceleration

   '''
   def cost(current_lataccel, target_lataccel, future_lataccel, future_lataccel_predict):
      
      # Compute current error
      current_error = current_lataccel - target_lataccel

      # Compute future error
      future_error = np.sum(np.subtract(future_lataccel, future_lataccel_predict))

      # Total error
      total_error = current_error + future_error

      return total_error
   
   '''
   Here I will define the optimization for the steering
   over the next 10 steps
   '''
   def optimize():
      return 'optimal'
    

   '''
   PURPOSE: This function is called by the PhysicsEngine to 
   increment the lateral acceleration by the steering 
   command 

   target_lataccel: is a float for the vehicles target lateral acceleration

   current_lataccel: is a float for the vehicles current lateral acceleration

   state: is a named tuple with [roll_lataccel, v_ego, a_ego]
   of the car for the current time period
   
   future_plan: is a named tuple with [lataccel,  roll_lataccel, v_ego, a_ego]
   for the next 10 steps

   '''
   def update(self, target_lataccel, current_lataccel, state, future_plan):
      target_lataccel 

      current_lataccel

      # Breaking apart state
      current_roll_lataccel = state.roll_lataccel
      current_velocity = state.v_ego
      current_acceleration = state.a_ego 

      # Breaking apart future_plan
      future_lataccel = future_plan.lataccel
      #future_plan.roll_lataccel # I don't suspect I'll use this 
      future_plan.v_ego
      future_plan.a_ego

      # Predict future lateral acceleration 
      future_lataccel_predict = self.predict(current_lataccel,
                                             current_roll_lataccel,
                                             current_velocity, 
                                             current_acceleration)

      # Difference between prediction and target lateral acceleration 
      cost = self.cost(current_lataccel, target_lataccel, future_lataccel)




      # Computing the steering angle 
      steering_angle = 0.0

      return steering_angle
   
