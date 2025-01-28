from . import BaseController
import numpy as np
from random import uniform
from scipy import optimize

class Controller(BaseController):
   """
   Initialize simplified vehicle model using roll dynamics
   
   Args:
      mass (float): Vehicle mass [kg]
      roll_center_height (float): Height of roll center from ground [m]
      roll_stiffness (float): Total roll stiffness [Nm/rad]
      roll_damping (float): Roll damping coefficient [Nms/rad]
      wheelbase (float): Vehicle wheelbase [m]
      steering_ratio (float): Steering ratio (steering wheel to road wheel)
   """
   def __init__(self, mass = 1500,
                  roll_center_height = 0.5,
                  roll_stiffness = 36000,
                  roll_damping = 4000,
                  wheelbase = 2.7,
                  steering_ratio = 15.0):
      self.mass = mass
      self.h_rc = roll_center_height
      self.K_roll = roll_stiffness
      self.C_roll = roll_damping
      self.wheelbase = wheelbase
      self.steering_ratio = steering_ratio
      self.g = 9.81  # gravitational acceleration
      

   """
   Aim:
      The system dynamics of the car will likely be varied
      since the data is from a bunch of different cars in 
      a bunch of different conditions. SOURCE OF ERRORS

   Args:
      steering_angle (float): Steering wheel angle [rad]
      current_lat_acc (float): Current lateral acceleration [m/s^2]
      roll_lat_acc (float): Lateral acceleration due to roll [m/s^2]
      ego_velocity (float): Vehicle forward velocity [m/s]
      ego_acceleration (float): Vehicle forward acceleration [m/s^2]
      dt (float): Time step [s]
         
   Returns:
      float: Predicted lateral acceleration [m/s^2]
   """
   def dynamics(self, steering_angle, current_lat_acc, roll_lat_acc, 
                                    ego_velocity, ego_acceleration, dt):
         # Convert steering wheel angle to road wheel angle
         road_wheel_angle = steering_angle / self.steering_ratio
         
         # Calculate steering-induced lateral acceleration (bicycle model approximation)
         # Using simplified steady-state relationship
         if abs(ego_velocity) > 0.1:  # Prevent division by zero
            steering_lat_acc = (ego_velocity**2 * road_wheel_angle) / self.wheelbase
         else:
            steering_lat_acc = 0
            
         # Calculate roll moment including steering effects
         total_lat_acc = current_lat_acc + steering_lat_acc
         roll_moment = self.mass * self.h_rc * total_lat_acc
         
         # Calculate roll angle and roll rate
         roll_angle = roll_moment / self.K_roll
         roll_rate = roll_angle / dt
         
         # Calculate roll damping effect
         damping_moment = self.C_roll * roll_rate
         
         # Combine all effects for lateral acceleration prediction
         predicted_lat_acc = (total_lat_acc + 
                           roll_lat_acc +
                           (damping_moment / (self.mass * self.h_rc)))
         
         return predicted_lat_acc


   '''
   This function is responsible for generating the 
   next 10 steps of current lateral acceleration,
   based on the system dynamics model 
   '''
   def predict(self, steer_command, current_lataccel):
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
      future_error = np.sum(np.subtract(future_lataccel_predict, future_lataccel)) # Not sure if the arguments are the right way around? 

      # Total error
      total_error = current_error + future_error

      return total_error
   
   
   '''
   Here I will define the optimization for the steering
   over the next 10 steps
   '''
   def optimize(self, current_lataccel, target_lataccel, state, future_plan):

      # Future plan arrays
      future_lataccel_targets = future_plan.lataccel[:10] # currently just limiting to 10 
      future_roll_lataccel = future_plan.roll_lataccel[:10]
      future_velo = future_plan.v_ego[:10]
      future_accel = future_plan.a_ego[:10]
      
      # State variables
      current_roll_lataccel = state.roll_lataccel
      current_velo = state.v_ego
      current_accel = state.a_ego


      # Generate empty steer command array
      steer_commands = np.zeros(len(future_lataccel_targets))

      
      # compute the minimal value 
      optimal_steering = optimize.minimize(self.cost,
                              steer_commands,
                              args=(current_lataccel, 
                                    target_lataccel,
                                    future_lataccel_targets),
                              method='nelder-mead')
      
      # Return only the next steering command 
      try:
         optimal_steer = optimal_steering.x[0]
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

      # Calling optimize 
      optimal_steer_command = self.optimize(current_lataccel, 
                                            target_lataccel, 
                                            state, 
                                            future_plan)


      return optimal_steer_command
   
