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
        racing_track = [[0.00307, -3.35292, 4.0, 0.07534],
[0.26724, -3.49795, 4.0, 0.07534],
[0.53142, -3.64297, 4.0, 0.07534],
[0.79559, -3.788, 4.0, 0.07534],
[1.05976, -3.93303, 4.0, 0.07534],
[1.32393, -4.07807, 4.0, 0.07534],
[1.5881, -4.22309, 4.0, 0.07534],
[1.85229, -4.3681, 4.0, 0.07534],
[2.11648, -4.51309, 4.0, 0.07534],
[2.38068, -4.65806, 4.0, 0.07534],
[2.64484, -4.80313, 4.0, 0.07534],
[2.90894, -4.94831, 4.0, 0.07534],
[3.17307, -5.09342, 3.57903, 0.0842],
[3.43718, -5.23859, 3.01184, 0.10006],
[3.7012, -5.38393, 2.64878, 0.11378],
[3.96537, -5.52786, 2.39085, 0.12583],
[4.2302, -5.66725, 2.19132, 0.13658],
[4.49607, -5.79906, 2.03193, 0.14604],
[4.76313, -5.92036, 1.90157, 0.15425],
[5.03137, -6.02846, 1.79174, 0.1614],
[5.30054, -6.12087, 1.69363, 0.16804],
[5.57022, -6.19536, 1.60706, 0.1741],
[5.83981, -6.24984, 1.53086, 0.17966],
[6.1085, -6.28244, 1.53086, 0.1768],
[6.37524, -6.29136, 1.53086, 0.17434],
[6.63873, -6.27471, 1.53086, 0.17246],
[6.89725, -6.23051, 1.53086, 0.17132],
[7.14848, -6.15663, 1.6037, 0.16329],
[7.39134, -6.05613, 1.63915, 0.16035],
[7.62454, -5.93029, 1.67403, 0.15829],
[7.84677, -5.7802, 1.71059, 0.15677],
[8.05669, -5.60683, 1.74651, 0.15589],
[8.25284, -5.41113, 1.78453, 0.15527],
[8.43368, -5.19422, 1.81966, 0.1552],
[8.5975, -4.95737, 1.81612, 0.15857],
[8.7425, -4.70236, 1.78142, 0.16467],
[8.86682, -4.43166, 1.74827, 0.17039],
[8.96768, -4.14856, 1.71331, 0.17541],
[9.04206, -3.85824, 1.68005, 0.17838],
[9.08815, -3.56658, 1.64714, 0.17927],
[9.10569, -3.27789, 1.61415, 0.17918],
[9.09526, -2.99526, 1.5812, 0.17887],
[9.05766, -2.72108, 1.54328, 0.17933],
[8.99379, -2.45731, 1.5, 0.18093],
[8.9045, -2.20571, 1.5, 0.17798],
[8.79049, -1.96802, 1.5, 0.17575],
[8.65233, -1.74605, 1.5, 0.17431],
[8.49029, -1.54197, 1.5, 0.17372],
[8.30446, -1.35856, 1.56837, 0.16648],
[8.09805, -1.19581, 1.64454, 0.15983],
[7.87383, -1.0534, 1.73037, 0.15351],
[7.63416, -0.93084, 1.82829, 0.14723],
[7.38121, -0.82743, 1.94167, 0.14074],
[7.11695, -0.74223, 2.07639, 0.13372],
[6.84326, -0.67403, 2.24308, 0.12575],
[6.5619, -0.62128, 2.45493, 0.11661],
[6.27453, -0.58211, 2.73947, 0.10587],
[5.98271, -0.55431, 3.15089, 0.09303],
[5.6879, -0.53539, 3.24838, 0.09094],
[5.39134, -0.52267, 3.03695, 0.09774],
[5.09506, -0.50503, 2.86805, 0.10349],
[4.79976, -0.4813, 2.74484, 0.10793],
[4.50573, -0.45026, 2.63387, 0.11226],
[4.2133, -0.41073, 2.55296, 0.11559],
[3.92287, -0.36158, 2.48709, 0.11843],
[3.63496, -0.30172, 2.43512, 0.12076],
[3.35015, -0.23032, 2.39897, 0.1224],
[3.06917, -0.14656, 2.3779, 0.1233],
[2.79283, -0.04986, 2.36715, 0.12368],
[2.52205, 0.06015, 2.36715, 0.12347],
[2.25775, 0.18368, 2.36715, 0.12324],
[2.00088, 0.3207, 2.36715, 0.12299],
[1.75228, 0.47096, 2.36715, 0.12272],
[1.51267, 0.63406, 2.36882, 0.12236],
[1.28267, 0.80943, 2.38828, 0.12111],
[1.06263, 0.99638, 2.41712, 0.11945],
[0.85278, 1.19415, 2.45822, 0.1173],
[0.65316, 1.40195, 2.52072, 0.11431],
[0.4636, 1.61894, 2.58643, 0.1114],
[0.28388, 1.84433, 2.67266, 0.10786],
[0.11362, 2.07735, 2.77705, 0.10392],
[-0.04767, 2.31722, 2.90329, 0.09956],
[-0.20058, 2.56319, 3.05137, 0.09492],
[-0.34575, 2.81455, 3.22332, 0.09005],
[-0.48385, 3.07065, 3.44489, 0.08446],
[-0.61567, 3.33081, 3.36733, 0.08661],
[-0.74194, 3.59443, 2.79008, 0.10477],
[-0.86347, 3.86092, 2.43278, 0.1204],
[-0.96522, 4.09287, 2.18377, 0.11598],
[-1.06956, 4.32277, 1.99213, 0.12673],
[-1.17861, 4.54896, 1.84289, 0.13626],
[-1.29441, 4.76993, 1.71846, 0.14517],
[-1.41896, 4.98421, 1.71846, 0.14423],
[-1.55423, 5.1904, 1.71846, 0.1435],
[-1.70224, 5.38701, 1.71846, 0.1432],
[-1.86506, 5.5723, 1.71846, 0.14354],
[-2.04482, 5.74398, 1.75706, 0.14147],
[-2.24064, 5.90168, 1.81166, 0.13878],
[-2.4514, 6.0451, 1.86734, 0.13652],
[-2.67605, 6.17393, 1.92466, 0.13455],
[-2.91357, 6.28785, 1.98751, 0.13254],
[-3.16295, 6.38659, 2.05329, 0.13063],
[-3.42316, 6.46995, 2.12519, 0.12857],
[-3.69314, 6.53785, 2.199, 0.1266],
[-3.97182, 6.59024, 2.28043, 0.12435],
[-4.25807, 6.62721, 2.3669, 0.12194],
[-4.55071, 6.64898, 2.40536, 0.122],
[-4.84844, 6.65594, 2.33161, 0.12773],
[-5.14968, 6.64861, 2.26358, 0.13312],
[-5.44612, 6.62737, 2.19335, 0.1355],
[-5.7368, 6.59194, 2.12852, 0.13757],
[-6.02107, 6.54207, 2.06406, 0.13983],
[-6.29838, 6.47754, 2.00276, 0.14216],
[-6.56818, 6.39813, 1.92057, 0.14644],
[-6.82988, 6.30353, 1.90765, 0.14587],
[-7.08284, 6.1934, 1.8947, 0.14561],
[-7.32628, 6.0673, 1.88003, 0.14583],
[-7.55933, 5.92471, 1.86545, 0.14646],
[-7.78061, 5.76462, 1.85092, 0.14756],
[-7.98932, 5.58715, 1.83572, 0.14924],
[-8.18444, 5.39238, 1.81937, 0.15154],
[-8.36468, 5.18037, 1.80233, 0.15439],
[-8.52839, 4.9514, 1.78079, 0.15806],
[-8.67354, 4.70621, 1.75843, 0.16204],
[-8.79773, 4.44635, 1.73468, 0.16603],
[-8.89843, 4.17459, 1.70753, 0.16973],
[-8.97338, 3.89492, 1.68149, 0.17219],
[-9.02109, 3.61197, 1.64765, 0.17416],
[-9.04114, 3.33002, 1.61461, 0.17507],
[-9.03389, 3.05239, 1.57924, 0.17586],
[-8.99995, 2.78144, 1.57924, 0.17291],
[-8.93995, 2.51889, 1.57924, 0.17054],
[-8.85409, 2.26626, 1.57924, 0.16896],
[-8.74227, 2.0251, 1.57924, 0.16832],
[-8.60399, 1.79726, 1.71731, 0.1552],
[-8.44358, 1.58186, 1.81448, 0.14801],
[-8.26329, 1.37863, 1.93002, 0.14076],
[-8.06523, 1.18713, 2.06645, 0.13332],
[-7.85142, 1.00676, 2.23292, 0.12528],
[-7.62387, 0.83669, 2.44628, 0.11613],
[-7.38467, 0.67585, 2.73434, 0.10542],
[-7.13596, 0.52285, 3.15279, 0.09262],
[-6.87993, 0.376, 3.83517, 0.07696],
[-6.61872, 0.2334, 4.0, 0.0744],
[-6.35464, 0.09302, 4.0, 0.07477],
[-6.08848, -0.04807, 4.0, 0.07531],
[-5.82245, -0.18943, 4.0, 0.07531],
[-5.55655, -0.33104, 4.0, 0.07531],
[-5.29078, -0.4729, 4.0, 0.07532],
[-5.02512, -0.61499, 4.0, 0.07532],
[-4.75958, -0.75731, 4.0, 0.07532],
[-4.49416, -0.89987, 4.0, 0.07532],
[-4.22882, -1.04259, 4.0, 0.07532],
[-3.9636, -1.18555, 4.0, 0.07532],
[-3.6985, -1.32874, 4.0, 0.07533],
[-3.43352, -1.47217, 4.0, 0.07533],
[-3.16866, -1.61584, 4.0, 0.07533],
[-2.90392, -1.75974, 4.0, 0.07533],
[-2.63929, -1.90388, 4.0, 0.07533],
[-2.37479, -2.04826, 4.0, 0.07534],
[-2.11041, -2.19287, 4.0, 0.07534],
[-1.84615, -2.33773, 4.0, 0.07534],
[-1.58198, -2.48276, 4.0, 0.07534],
[-1.3178, -2.62779, 4.0, 0.07534],
[-1.05363, -2.77282, 4.0, 0.07534],
[-0.78946, -2.91785, 4.0, 0.07534],
[-0.52528, -3.06287, 4.0, 0.07534],
[-0.26111, -3.2079, 4.0, 0.07534]]

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
        STANDARD_TIME = 35
        FASTEST_TIME = 25
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
        STANDARD_TIME = 35  # seconds (time that is easily done by model)
        FASTEST_TIME = 25  # seconds (best time of 1st place on the track)
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
