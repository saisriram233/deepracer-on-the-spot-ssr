import math

def reward_function(params):
    STRAIGHT = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 40, 41, 42, 43, 44, 45, 46, 47,
                   48, 55, 56, 57, 58, 59, 60, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101,
                   110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130]
    LEFT =  [23, 24,25,26, 50, 51, 52, 61, 62, 63, 64,  131, 132, 133, 134, 135, 136, 107, 108, 109]
    RIGHT = [35, 36, 37, 38, 39, 77,78,79,80]
    STRAIGHT_SPEED = 3
    TURN_SPEED = 2.5
    center_lane_threshold = 0.5
    reward = 1.0

    # Read input variables
    x = params['x']
    y = params['y']
    speed = params['speed']
    heading = params['heading']
    waypoints = params['waypoints']
    is_offtrack = params['is_offtrack']
    steering = params['steering_angle']
    closest_waypoints = params['closest_waypoints']
    all_wheels_on_track = params['all_wheels_on_track']

    if is_offtrack:
        return 1e-3  # Minimum reward for being offtrack

    if not all_wheels_on_track:
        return 1e-3  # Minimum reward if not all wheels are on track

    # Determine next waypoint
    next_waypoint_index = closest_waypoints[1]

    # Reward based on center lane
    if next_waypoint_index in STRAIGHT:
        # Reward high speed without penalizing
        if speed >=STRAIGHT_SPEED:
          reward += 2  # Proportional reward based on speed
        # Reward for low steering angle
        if steering < 5 or steering > -5 :
            reward += 2  # Reward for maintaining a low steering angle
        else:
            reward -= 1  # Penalize high steering angle
       
    else:
        if next_waypoint_index in LEFT:
          if steering > 0:
            reward += 2
          else:
            reward -= 1
        elif next_waypoint_index in RIGHT:
          if steering < 0:
            reward += 2
          else:
            reward -= 1

        
        # Penalize high speed in turns
        if speed < TURN_SPEED:
            reward += 2  # Penalize for high speed when not in center lane

    

    # Final reward
    reward = max(reward, 1e-3)  # Ensure reward is not too low
    return float(reward)
