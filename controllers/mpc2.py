from . import BaseController
import numpy as np
from random import uniform
from scipy import optimize

class Controller(BaseController):
   def __init__(self):
      "something"

   '''
   The system dynamics of the car will likely be varied
   since the data is from a bunch of different cars in 
   a bunch of different conditions. SOURCE OF ERRORS
   '''
   def dynamics(self, lataccel, steering):
      return lataccel + steering  


   '''
   This function is responsible for generating the 
   next 10 steps of current lateral acceleration,
   based on the system dynamics model 
   '''
   def predict(self, steer_command, current_lataccel):
      # We need to generate some steering angle for the next 10 steps
      # steer_command = [(uniform(-1, 1)) for i in range(10)]
      #future_lataccel_predict = np.zeros(10)

      future_lataccel_predict = np.zeros(len(steer_command))

      '''
      Generate future lataccel by adding the steering commands off of 
      the last steer command.
      '''
      for i in range(len(steer_command)):
         # First prediction 
         if i == 0:
            future_lataccel_predict[i] = current_lataccel + steer_command[i]
         else: # The rest
            future_lataccel_predict[i] = self.dynamics(future_lataccel_predict[i-1], steer_command[i])

      return future_lataccel_predict


   '''
   Here we will evaluate the cost of the predictions 
   vs the target lateral acceleration

   '''
   def cost(self, steer_commands, current_lataccel, target_lataccel, future_lataccel):
      # Compute current error
      current_error = current_lataccel - target_lataccel

      # Generate the future predictions
      future_lataccel_predict = self.predict(steer_commands, current_lataccel)

      # Compute future error
      #future_error = np.sum(np.subtract(future_lataccel_predict, future_lataccel[:10])) # Not sure if the arguments are the right way around? 
      future_error = np.sum(np.subtract(future_lataccel_predict, future_lataccel)) # Not sure if the arguments are the right way around? 

      # Total error
      total_error = current_error + future_error

      return total_error
   

   '''
   Here I will define the optimization for the steering
   over the next 10 steps
   '''
   def optimize(self, current_lataccel, target_lataccel, future_lataccel):
      # Generate random steering commands
      #steer_commands = np.zeros(10)

      # Generate random steering commands, the size of future_lataccel
      steer_commands = np.zeros(len(future_lataccel))

      # compute the minimal value 
      res = optimize.minimize(self.cost,
                              steer_commands,
                              args=(current_lataccel, 
                                    target_lataccel,
                                    future_lataccel),
                              method='nelder-mead')
      
      # Return only the next steering command 
      try:
         optimal_steer = res.x[0]
      except: 
         return 0.0

      return optimal_steer


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
      # Break apart state
      current_roll_lataccel = state.roll_lataccel
      current_velocity = state.v_ego
      current_acceleration = state.a_ego 

      # Break apart future_plan
      future_lataccel = future_plan.lataccel
      #future_plan.roll_lataccel # I don't suspect I'll use this 
      future_plan.v_ego
      future_plan.a_ego

      # Calling optimize 
      optimal_steer_command = self.optimize(current_lataccel, target_lataccel, future_lataccel)

      return optimal_steer_command
   
