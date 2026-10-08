import numpy as np
import matplotlib.pyplot as plt


def make_car(desired_v:float=20.0, dt:float=0.1) -> dict:
    """ 
    Generates a dictionary that holds all the car's values. Keeps track of state varaibles.
    """
    car_state_dictionary : dict[str, float] = {
        "v" : 0, #velocity of your car 
        "a" : 0, #acceleration of your car
        "t" : 0, #time of your car
        "x" : 0, #position of your car
        "dt" : dt, #time step of your car, how much the time changes every time you update/step
        "desired_v" : desired_v, #desired velocity of your car, the velocity you want to maintain
        "step" : 0,
    
        #hint: use these variables in the integral and derivative portion of your PID control (steps 5 and 6 )
        "error_prev" : None,
        "net_integral" : 0.0
    }
    return car_state_dictionary

def update(car: dict, throttle_perc: float, mass: float = 1000, max_throttle_force: float = 5000, friction: float = 2.0) -> None:
        """
        Updates the car's state variables based on the throttle percentage.
        Use this function after finding throttle percentage to update the car's state variables.

        Inputs:
        car: dictionary containing the car's state variables
        throttle_perc: float, throttle percentage (-1 to 1)

        Outputs:
        None, but updates the car's state variables
        """
        force = throttle_perc * max_throttle_force
        car["a"] = (force / mass) - friction
        car["v"] += car["a"] * car["dt"]
        car["x"] += car["v"] * car["dt"]
        car["t"] += car["dt"]
        car["step"] += 1


def calculate_desired_acceleration(car: dict, K_P: float, K_I: float = 0.0, K_D: float = 0.0) -> tuple[float, float]:
        #input: car["v"], car["desired_v"] (floats)
        #output: desired acceleration and error tuple(float, float)
       
        dt = car["dt"]
        error = car["desired_v"] - car["v"] # error is the target speed minus the current speed
        p_term = K_P * error # proportional term... K_P * error because bigger error means bigger push 
        
        car["net_integral"] += error * dt #sum of all errors * dt to get area under the curve
        i_term = K_I * car["net_integral"]

        if K_I > 0.0:
               limit = 5 / K_I # limit the integral term to prevent windup
               car["net_integral"] = float(np.clip(car["net_integral"], -limit, limit)) # clip the integral term to prevent windup

        if car["error_prev"] is None:
                d_term = 0.0
        else:
                d_term = K_D * (error - car["error_prev"]) / dt # derivative term... K_D * (error - prev_error) / dt because bigger change in error means bigger push
        car["error_prev"] = error # store the current error  

        acceleration_desired = p_term + i_term + d_term # sum of all terms to get desired acceleration
        
        return(acceleration_desired, error)

def acceleration_to_throttle_percentage(acceleration_desired: float, mass: float = 1000, max_throttle_force: float = 5000) -> float:
        #input: desired_acceleration(float)
        #output: throttle percentage (float, -1 to 1)
       max_acceleration = max_throttle_force / mass 
       # the max acceleration is the max force divided by the mass because F = ma, so a = F/m
       throttle_percentage = acceleration_desired / max_acceleration 
       # the throttle percentage is the desired acceleration divided by the max acceleration
       # because if you want to go faster than the max, you need to give more throttle than 100%
       # which is not possible, so we clip it to 100%
       return float(np.clip(throttle_percentage, -1.0, 1.0))
