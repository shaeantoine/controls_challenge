from . import BaseController
import numpy as np
from scipy.optimize import minimize  

'''
class Controller(BaseController):
    def __init__(self): 
        self.prediction_horizon = 10 
        self.lambda_u = 1
        self.gamma = 1  # Regularization term for system response
        self.a = 0.9  # Coefficient for current_lataccel
        self.b = 0.1  # Coefficient for delta_u


    def dynamic_model(self, current_lataccel, delta_u): 
        return self.a * current_lataccel + self.b * delta_u
    
    def cost_function(self, u_sequence, target_lataccel, current_lataccel, future_plan): 
        lataccel_pred = current_lataccel 
        total_cost = 0

        # Initial cost 
        initial_error = target_lataccel - current_lataccel
        total_cost += initial_error ** 2

        # Cost at future time steps 
        for i, j in zip(range(self.prediction_horizon), future_plan): 
            delta_u = u_sequence[i]
            lataccel_pred = self.dynamic_model(lataccel_pred, delta_u) 

            tracking_error = lataccel_pred - j # Note funny index stuff 
            total_cost += tracking_error ** 2

            total_cost += self.lambda_u * (delta_u ** 2)

            expected_lataccel = self.a * lataccel_pred + self.b * delta_u
            total_cost += self.gamma * (lataccel_pred - expected_lataccel) ** 2
        
        return total_cost

    def update(self, target_lataccel, current_lataccel, state, future_plan):
        u_initial = np.zeros(self.prediction_horizon)

        result = minimize(
            self.cost_function,
            u_initial,
            args=(target_lataccel, current_lataccel, future_plan.lataccel),
            method='SLSQP', 
            bounds=[(-2,2)] * self.prediction_horizon
        )

        optimal_u_sequence = result.x
        optimal_delta_u = optimal_u_sequence[0]

        return optimal_delta_u

'''



class Controller(BaseController):
    def __init__(self, dt=0.1, horizon=10):
        """
        Initialize MPC controller for lateral acceleration tracking
        
        Args:
            dt: Time step between predictions (seconds)
            horizon: Number of steps to predict into future
        """
        self.dt = dt
        self.horizon = horizon
        
        # Controller parameters
        self.max_steering_rate = np.deg2rad(100)  # deg/s
        
        # Weights for different terms in cost function
        self.lat_accel_error_weight = 1.0
        self.steering_rate_weight = 0.1
        self.steering_accel_weight = 0.01

        # Future steps 
        self.N = 10

    
    def predict_lataccel(self, steering_sequence, v_ego_seq, roll_lataccel_seq, current_lataccel):
        """
        Predict lateral accelerations for a sequence of steering rates
        """
        wheelbase = 2.7  # meters

        steering_angles = np.zeros(self.N)
        lat_accels = np.zeros(self.N)

        # Back-calculate initial steering angle from current lateral acceleration
        # current_lataccel = v^2 * initial_angle / wheelbase + roll_lataccel
        initial_roll_component = roll_lataccel_seq[0]
        initial_steering_component = current_lataccel - initial_roll_component
        current_angle = (initial_steering_component * wheelbase) / (v_ego_seq[0]**2) if v_ego_seq[0] > 0.1 else 0.0
        
        # First prediction starts from current state
        lat_accels[0] = current_lataccel
        
        for i in range(self.N):
            # Update steering angle
            current_angle = current_angle + steering_sequence[i] * self.dt
            steering_angles[i] = current_angle
            
            # Calculate next lateral acceleration
            if i > 0:  # Skip first one as we use current_lataccel
                steering_component = v_ego_seq[i]**2 * steering_angles[i] / wheelbase
                roll_component = roll_lataccel_seq[i]
                lat_accels[i] = steering_component + roll_component
            
        return lat_accels
    
    
    def objective(self, steering_rates, target_lataccels, roll_lataccels, v_egos, current_lataccel):
        """
        Compute cost for a sequence of steering rate commands
        """
        steering_rates = steering_rates.reshape(-1)[:self.N]  # Ensure steering_rates matches other sequences
        
        predicted_lataccels = self.predict_lataccel(steering_rates, 
                                                    v_egos, 
                                                    roll_lataccels,
                                                    current_lataccel)
        
        cost = 0.0
        for t in range(self.N):
            accel_error = predicted_lataccels[t] - target_lataccels[t]
            cost += self.lat_accel_error_weight * accel_error**2
            cost += self.steering_rate_weight * steering_rates[t]**2
            
            if t > 0: 
                steering_accel = (steering_rates[t] - steering_rates[t-1]) / self.dt
                cost += self.steering_accel_weight * steering_accel**2
        
        return float(cost)


    def update(self, target_lataccel, current_lataccel, state, future_plan):
        """
        Compute optimal steering rate command based on current state and future plan
        
        Args:
            target_lataccel: Current target lateral acceleration
            current_lataccel: Current lateral acceleration
            state: State named tuple with (roll_lataccel, v_ego, a_ego)
            future_plan: List of FuturePlan named tuples, each containing
                        (lataccel, roll_lataccel, v_ego, a_ego)
        
        Returns:
            Optimal steering rate command
        """
        # Get future steps
        self.N = min(len(future_plan.lataccel) + 1, self.horizon) 

        # Extract sequences for prediction
        target_lataccels = np.array([target_lataccel] + [plan for plan in future_plan.lataccel[:self.N-1]])
        roll_lataccels = np.array([state.roll_lataccel] + [plan for  plan in future_plan.roll_lataccel[:self.N-1]])
        v_egos = np.array([state.v_ego] + [plan for plan in future_plan.v_ego[:self.N-1]])
        
        # Initial guess - zero steering rates
        steering_rates_guess = np.zeros(self.N)
        
        # Optimization bounds
        # Low impact on outcome, assuming its fine like this
        #bounds = [(-self.max_steering_rate, self.max_steering_rate)] * self.N
        bounds = [(-10,10)] * self.N
        
        # Optimize steering rates
        result = minimize(
            self.objective,
            steering_rates_guess,
            args=(target_lataccels, roll_lataccels, v_egos, current_lataccel),
            method='SLSQP',
            bounds=bounds
        )
        
        # Return first optimal steering rate
        return float(result.x[0])