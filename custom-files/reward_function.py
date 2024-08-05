def reward_function(params):
    '''
    Example of rewarding the agent to follow center line
    '''
    
    if params["is_offtrack"] == False and params["steps"] > 0:

        reward = ((params["progress"] / params["steps"]) * 100) + (params["speed"]**2)

    else:

        reward = 0.01

    return float(reward)  
