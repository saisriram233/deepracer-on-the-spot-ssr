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
        racing_track =  [[6.50751, 2.93214, 1.16739, 0.14703],
                        [6.64719, 2.91544, 1.16739, 0.12051],
                        [6.80764, 2.89294, 1.16739, 0.13878],
                        [7.00146, 2.85824, 1.16739, 0.16867],
                        [7.22463, 2.80389, 1.16739, 0.19675],
                        [7.4542, 2.72533, 1.16739, 0.20785],
                        [7.66925, 2.6221, 1.16739, 0.20433],
                        [7.85693, 2.49425, 1.16739, 0.19453],
                        [8.00893, 2.34213, 1.16739, 0.18421],
                        [8.11881, 2.16647, 1.16739, 0.17748],
                        [8.1756, 1.96726, 1.3649, 0.15177],
                        [8.19032, 1.75357, 1.49784, 0.143],
                        [8.16498, 1.52905, 1.6777, 0.13468],
                        [8.10196, 1.29707, 1.92068, 0.12516],
                        [8.00504, 1.0608, 2.17043, 0.11766],
                        [7.87745, 0.82284, 2.51769, 0.10724],
                        [7.72439, 0.5854, 2.95497, 0.0956],
                        [7.55158, 0.35, 3.55862, 0.08206],
                        [7.36494, 0.1174, 3.8, 0.07848],
                        [7.16834, -0.11059, 3.8, 0.07922],
                        [6.96524, -0.33229, 3.8, 0.07912],
                        [6.75701, -0.54779, 3.8, 0.07886],
                        [6.54446, -0.75738, 3.8, 0.07855],
                        [6.32804, -0.96117, 3.8, 0.07823],
                        [6.10795, -1.15911, 3.61133, 0.08196],
                        [5.88435, -1.35108, 3.21038, 0.0918],
                        [5.65734, -1.53699, 2.86273, 0.1025],
                        [5.42691, -1.71653, 2.86273, 0.10204],
                        [5.19301, -1.88934, 2.86273, 0.10159],
                        [4.9556, -2.0551, 2.86273, 0.10115],
                        [4.71473, -2.21366, 2.86273, 0.10073],
                        [4.47048, -2.36496, 2.86273, 0.10036],
                        [4.22295, -2.50901, 2.86273, 0.10004],
                        [3.97193, -2.64509, 2.86273, 0.09974],
                        [3.71706, -2.77206, 2.86273, 0.09947],
                        [3.45786, -2.88842, 2.86273, 0.09925],
                        [3.19346, -2.99132, 2.86273, 0.09911],
                        [2.92302, -3.07715, 3.12809, 0.0907],
                        [2.64806, -3.14847, 3.40844, 0.08334],
                        [2.36971, -3.2074, 3.64715, 0.07801],
                        [2.0888, -3.25538, 3.8, 0.07499],
                        [1.80591, -3.29328, 3.8, 0.07511],
                        [1.52148, -3.32126, 3.8, 0.07521],
                        [1.23603, -3.34005, 3.8, 0.07528],
                        [0.94996, -3.34972, 3.8, 0.07532],
                        [0.6637, -3.35022, 3.5171, 0.08139],
                        [0.37769, -3.34151, 3.18045, 0.08997],
                        [0.0924, -3.32363, 3.18045, 0.08988],
                        [-0.19171, -3.29632, 3.18045, 0.08974],
                        [-0.47417, -3.25966, 3.18045, 0.08956],
                        [-0.75466, -3.21403, 3.18045, 0.08935],
                        [-1.033, -3.16017, 3.18045, 0.08914],
                        [-1.30897, -3.0983, 3.18045, 0.08892],
                        [-1.5824, -3.0288, 3.18045, 0.08871],
                        [-1.85282, -2.95107, 3.18045, 0.08847],
                        [-2.11956, -2.86425, 2.6711, 0.10502],
                        [-2.38149, -2.76672, 2.14601, 0.13024],
                        [-2.63687, -2.65655, 1.8035, 0.15422],
                        [-2.88827, -2.53964, 1.8035, 0.15373],
                        [-3.13611, -2.41712, 1.8035, 0.15329],
                        [-3.38075, -2.29, 1.8035, 0.15287],
                        [-3.62245, -2.15893, 1.8035, 0.15245],
                        [-3.86117, -2.02427, 1.8035, 0.15197],
                        [-4.09673, -1.88625, 1.8035, 0.15138],
                        [-4.31463, -1.75345, 1.8035, 0.14149],
                        [-4.53451, -1.62864, 1.8035, 0.14019],
                        [-4.75838, -1.5201, 1.8035, 0.13795],
                        [-4.98797, -1.43561, 1.80125, 0.13581],
                        [-5.22479, -1.38403, 1.60299, 0.1512],
                        [-5.4667, -1.35836, 1.4294, 0.17019],
                        [-5.71252, -1.35456, 1.25044, 0.19661],
                        [-5.96156, -1.37046, 1.1, 0.22686],
                        [-6.21352, -1.40577, 1.1, 0.23129],
                        [-6.46823, -1.46043, 1.1, 0.23682],
                        [-6.7209, -1.49557, 1.1, 0.23191],
                        [-6.9661, -1.50893, 1.1, 0.22324],
                        [-7.20157, -1.49892, 1.1, 0.21426],
                        [-7.42514, -1.46454, 1.1, 0.20563],
                        [-7.63433, -1.40467, 1.1, 0.19781],
                        [-7.82515, -1.31659, 1.1, 0.19106],
                        [-7.99219, -1.19729, 1.1, 0.18661],
                        [-8.12558, -1.04148, 1.1, 0.18646],
                        [-8.20635, -0.84353, 1.33922, 0.15964],
                        [-8.24247, -0.62184, 1.45132, 0.15476],
                        [-8.233, -0.38653, 1.57052, 0.14995],
                        [-8.18017, -0.14953, 1.68802, 0.14385],
                        [-8.08951, 0.07906, 1.79686, 0.13686],
                        [-7.96806, 0.2935, 1.79686, 0.13715],
                        [-7.82096, 0.49073, 1.79686, 0.13693],
                        [-7.65264, 0.66956, 1.79686, 0.13668],
                        [-7.46626, 0.82951, 1.79686, 0.13669],
                        [-7.26392, 0.97002, 1.79686, 0.1371],
                        [-7.04682, 1.08999, 1.79686, 0.13804],
                        [-6.81101, 1.17926, 2.05973, 0.12242],
                        [-6.56194, 1.24317, 2.37223, 0.1084],
                        [-6.30307, 1.28653, 2.81474, 0.09325],
                        [-6.03725, 1.31436, 3.33052, 0.08025],
                        [-5.76692, 1.33177, 2.91344, 0.09298],
                        [-5.49436, 1.34388, 2.91344, 0.09365],
                        [-5.21461, 1.35859, 2.91344, 0.09615],
                        [-4.93518, 1.37617, 2.91344, 0.0961],
                        [-4.65623, 1.39725, 2.91344, 0.09602],
                        [-4.37797, 1.42259, 2.91344, 0.0959],
                        [-4.10071, 1.45314, 2.91344, 0.09574],
                        [-3.82484, 1.49, 2.91344, 0.09553],
                        [-3.55092, 1.53468, 2.91344, 0.09526],
                        [-3.27962, 1.58881, 2.91344, 0.09495],
                        [-3.01198, 1.65513, 2.91344, 0.09464],
                        [-2.74937, 1.73716, 2.95038, 0.09325],
                        [-2.4909, 1.83151, 2.42898, 0.11328],
                        [-2.23602, 1.93593, 2.42898, 0.1134],
                        [-1.98422, 2.04831, 2.42898, 0.11352],
                        [-1.73518, 2.16706, 2.42898, 0.11359],
                        [-1.48859, 2.29062, 2.42898, 0.11355],
                        [-1.24397, 2.4175, 2.42898, 0.11345],
                        [-1.00096, 2.54662, 2.42898, 0.11329],
                        [-0.76317, 2.67489, 2.42898, 0.11123],
                        [-0.52345, 2.79878, 2.42898, 0.11109],
                        [-0.27977, 2.91363, 2.42898, 0.11091],
                        [-0.03001, 3.01437, 2.42898, 0.11087],
                        [0.22853, 3.09406, 2.72637, 0.09923],
                        [0.49397, 3.15665, 3.00693, 0.0907],
                        [0.76507, 3.20478, 3.30447, 0.08333],
                        [1.04092, 3.24054, 3.61602, 0.07692],
                        [1.32075, 3.26566, 3.8, 0.07394],
                        [1.60398, 3.2815, 3.8, 0.07465],
                        [1.89013, 3.28926, 3.8, 0.07533],
                        [2.17876, 3.28999, 3.8, 0.07596],
                        [2.46949, 3.2847, 3.8, 0.07652],
                        [2.762, 3.2742, 3.8, 0.07703],
                        [3.05603, 3.25919, 3.8, 0.07748],
                        [3.35133, 3.24036, 3.8, 0.07787],
                        [3.64768, 3.21833, 3.8, 0.0782],
                        [3.94488, 3.19366, 3.70597, 0.08047],
                        [4.24276, 3.16685, 2.95801, 0.10111],
                        [4.54117, 3.13836, 2.52478, 0.11873],
                        [4.83998, 3.1086, 2.18979, 0.13713],
                        [5.13908, 3.07794, 1.94114, 0.15489],
                        [5.43836, 3.04667, 1.69017, 0.17804],
                        [5.73777, 3.01504, 1.47436, 0.2042],
                        [6.03727, 2.98309, 1.31396, 0.22923],
                        [6.33688, 2.95081, 1.16739, 0.25813]]

  
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


reward_object = Reward(verbose = True) # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
