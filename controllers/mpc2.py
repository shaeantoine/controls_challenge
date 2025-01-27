from . import BaseController


class controller(BaseController):
    def __init__(self):
        self.penis = "penis"

    '''
    The system dynamics of the car will likely be varied
    since the data is from a bunch of different cars in 
    a bunch of different conditions. SOURCE OF ERRORS


    '''
    def dynamics():
       return "shit"
    

    '''
    This function is responsible for generating the 
    next 10 steps of current lateral acceleration,
    based on the system dynamics model 


    '''
    def predict():
       return "predictions"
    

    '''
    Here we will evaluate the cost of the predictions 
    vs the target lateral acceleration

    '''
    def cost():
       return 'cost'   
    
    '''
    Here I will define the optimization for the steering
    over the next 10 steps
    '''
    def optimize():
       return 'optimal'
    

    '''
    This function is called by the PhysicsEngine to 
    increment the lateral acceleration by the steering 
    command 


    '''
    def update(self, target_lataccel, current_lataccel, state, future_plan):
      
      
      steering_angle = 0.0

      return steering_angle
