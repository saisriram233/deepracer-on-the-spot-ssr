import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track =  [[6.50751, 2.93214, 1.76467, 0.09727],
                        [6.64719, 2.91544, 1.53652, 0.09156],
                        [6.80764, 2.89294, 1.34032, 0.12088],
                        [7.00146, 2.85824, 1.19451, 0.16484],
                        [7.22463, 2.80389, 1.06127, 0.21643],
                        [7.4542, 2.72533, 1.06127, 0.22863],
                        [7.66925, 2.6221, 1.06127, 0.22477],
                        [7.85693, 2.49425, 1.06127, 0.21398],
                        [8.00893, 2.34213, 1.06127, 0.20264],
                        [8.11881, 2.16647, 1.06127, 0.19523],
                        [8.1756, 1.96726, 1.24082, 0.16694],
                        [8.19032, 1.75357, 1.36167, 0.1573],
                        [8.16498, 1.52905, 1.52518, 0.14814],
                        [8.10196, 1.29707, 1.74607, 0.13767],
                        [8.00504, 1.0608, 1.97312, 0.12943],
                        [7.87745, 0.82284, 2.28881, 0.11797],
                        [7.72439, 0.5854, 2.68633, 0.10516],
                        [7.55158, 0.35, 3.23511, 0.09027],
                        [7.36494, 0.1174, 3.5, 0.08521],
                        [7.16834, -0.11059, 3.5, 0.08601],
                        [6.96524, -0.33229, 3.5, 0.0859],
                        [6.75701, -0.54779, 3.5, 0.08562],
                        [6.54446, -0.75738, 3.5, 0.08529],
                        [6.32804, -0.96117, 3.5, 0.08494],
                        [6.10795, -1.15911, 3.5, 0.08457],
                        [5.88435, -1.35108, 3.5, 0.0842],
                        [5.65734, -1.53699, 3.5, 0.08383],
                        [5.42691, -1.71653, 3.5, 0.08346],
                        [5.19301, -1.88934, 3.5, 0.08309],
                        [4.9556, -2.0551, 3.28303, 0.0882],
                        [4.71473, -2.21366, 2.91852, 0.09881],
                        [4.47048, -2.36496, 2.60248, 0.1104],
                        [4.22295, -2.50901, 2.60248, 0.11005],
                        [3.97193, -2.64509, 2.60248, 0.10971],
                        [3.71706, -2.77206, 2.60248, 0.10942],
                        [3.45786, -2.88842, 2.60248, 0.10917],
                        [3.19346, -2.99132, 2.60248, 0.10902],
                        [2.92302, -3.07715, 2.84372, 0.09977],
                        [2.64806, -3.14847, 3.09858, 0.09167],
                        [2.36971, -3.2074, 3.31559, 0.08581],
                        [2.0888, -3.25538, 3.47527, 0.082],
                        [1.80591, -3.29328, 3.5, 0.08155],
                        [1.52148, -3.32126, 3.5, 0.08166],
                        [1.23603, -3.34005, 3.5, 0.08173],
                        [0.94996, -3.34972, 3.5, 0.08178],
                        [0.6637, -3.35022, 3.5, 0.08179],
                        [0.37769, -3.34151, 3.5, 0.08175],
                        [0.0924, -3.32363, 3.5, 0.08167],
                        [-0.19171, -3.29632, 3.5, 0.08155],
                        [-0.47417, -3.25966, 3.5, 0.08138],
                        [-0.75466, -3.21403, 3.19737, 0.08888],
                        [-1.033, -3.16017, 2.89132, 0.09806],
                        [-1.30897, -3.0983, 2.89132, 0.09782],
                        [-1.5824, -3.0288, 2.89132, 0.09758],
                        [-1.85282, -2.95107, 2.89132, 0.09731],
                        [-2.11956, -2.86425, 2.89132, 0.09702],
                        [-2.38149, -2.76672, 2.89132, 0.09667],
                        [-2.63687, -2.65655, 3.5, 0.07947],
                        [-2.88827, -2.53964, 3.5, 0.07922],
                        [-3.13611, -2.41712, 3.5, 0.07899],
                        [-3.38075, -2.29, 2.42828, 0.11354],
                        [-3.62245, -2.15893, 1.95092, 0.14093],
                        [-3.86117, -2.02427, 1.63954, 0.16717],
                        [-4.09673, -1.88625, 1.63954, 0.16652],
                        [-4.31463, -1.75345, 1.63954, 0.15564],
                        [-4.53451, -1.62864, 1.63954, 0.15421],
                        [-4.75838, -1.5201, 1.63954, 0.15175],
                        [-4.98797, -1.43561, 1.63954, 0.14921],
                        [-5.22479, -1.38403, 1.84552, 0.13133],
                        [-5.4667, -1.35836, 2.03228, 0.11971],
                        [-5.71252, -1.35456, 1.94578, 0.12635],
                        [-5.96156, -1.37046, 1.78868, 0.13951],
                        [-6.21352, -1.40577, 1.6375, 0.15537],
                        [-6.46823, -1.46043, 1.45726, 0.17876],
                        [-6.7209, -1.49557, 1.29945, 0.19632],
                        [-6.9661, -1.50893, 1.13676, 0.21602],
                        [-7.20157, -1.49892, 1.0, 0.23569],
                        [-7.42514, -1.46454, 1.0, 0.22619],
                        [-7.63433, -1.40467, 1.0, 0.21759],
                        [-7.82515, -1.31659, 1.0, 0.21017],
                        [-7.99219, -1.19729, 1.0, 0.20527],
                        [-8.12558, -1.04148, 1.0, 0.20511],
                        [-8.20635, -0.84353, 1.21747, 0.1756],
                        [-8.24247, -0.62184, 1.31939, 0.17024],
                        [-8.233, -0.38653, 1.42775, 0.16494],
                        [-8.18017, -0.14953, 1.53456, 0.15823],
                        [-8.08951, 0.07906, 1.65131, 0.14892],
                        [-7.96806, 0.2935, 1.63351, 0.15087],
                        [-7.82096, 0.49073, 1.63351, 0.15062],
                        [-7.65264, 0.66956, 1.63351, 0.15035],
                        [-7.46626, 0.82951, 1.63351, 0.15035],
                        [-7.26392, 0.97002, 1.63351, 0.15081],
                        [-7.04682, 1.08999, 1.63351, 0.15185],
                        [-6.81101, 1.17926, 1.87248, 0.13466],
                        [-6.56194, 1.24317, 2.15657, 0.11924],
                        [-6.30307, 1.28653, 2.55885, 0.10257],
                        [-6.03725, 1.31436, 3.20073, 0.0835],
                        [-5.76692, 1.33177, 3.5, 0.0774],
                        [-5.49436, 1.34388, 3.5, 0.07795],
                        [-5.21461, 1.35859, 3.5, 0.08004],
                        [-4.93518, 1.37617, 3.45888, 0.08095],
                        [-4.65623, 1.39725, 3.02774, 0.09239],
                        [-4.37797, 1.42259, 2.64858, 0.10549],
                        [-4.10071, 1.45314, 2.64858, 0.10532],
                        [-3.82484, 1.49, 2.64858, 0.10509],
                        [-3.55092, 1.53468, 2.64858, 0.10479],
                        [-3.27962, 1.58881, 2.64858, 0.10445],
                        [-3.01198, 1.65513, 2.64858, 0.10411],
                        [-2.74937, 1.73716, 2.97824, 0.09238],
                        [-2.4909, 1.83151, 3.28823, 0.08368],
                        [-2.23602, 1.93593, 3.5, 0.0787],
                        [-1.98422, 2.04831, 3.5, 0.07878],
                        [-1.73518, 2.16706, 3.35306, 0.08228],
                        [-1.48859, 2.29062, 2.68216, 0.10284],
                        [-1.24397, 2.4175, 2.20816, 0.12479],
                        [-1.00096, 2.54662, 2.20816, 0.12462],
                        [-0.76317, 2.67489, 2.20816, 0.12236],
                        [-0.52345, 2.79878, 2.20816, 0.1222],
                        [-0.27977, 2.91363, 2.20816, 0.122],
                        [-0.03001, 3.01437, 2.20816, 0.12196],
                        [0.22853, 3.09406, 2.47852, 0.10916],
                        [0.49397, 3.15665, 2.73358, 0.09976],
                        [0.76507, 3.20478, 3.00407, 0.09166],
                        [1.04092, 3.24054, 3.28729, 0.08461],
                        [1.32075, 3.26566, 3.5, 0.08027],
                        [1.60398, 3.2815, 3.5, 0.08105],
                        [1.89013, 3.28926, 3.5, 0.08179],
                        [2.17876, 3.28999, 3.5, 0.08247],
                        [2.46949, 3.2847, 3.5, 0.08308],
                        [2.762, 3.2742, 3.5, 0.08363],
                        [3.05603, 3.25919, 3.5, 0.08412],
                        [3.35133, 3.24036, 3.5, 0.08454],
                        [3.64768, 3.21833, 3.5, 0.0849],
                        [3.94488, 3.19366, 3.5, 0.08521],
                        [4.24276, 3.16685, 3.5, 0.08545],
                        [4.54117, 3.13836, 3.5, 0.08565],
                        [4.83998, 3.1086, 3.5, 0.0858],
                        [5.13908, 3.07794, 3.5, 0.0859],
                        [5.43836, 3.04667, 3.36906, 0.08932],
                        [5.73777, 3.01504, 2.6891, 0.11196],
                        [6.03727, 2.98309, 2.29525, 0.13123],
                        [6.33688, 2.95081, 1.99071, 0.15137]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
