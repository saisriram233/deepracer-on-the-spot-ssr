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
        racing_track =  [[6.81348, 2.86624, 2.85658, 0.0816],
                        [6.57754, 2.91002, 3.32747, 0.07212],
                        [6.33498, 2.94648, 3.8, 0.06455],
                        [6.08812, 2.97783, 3.8, 0.06548],
                        [5.83892, 3.00624, 3.8, 0.06601],
                        [5.58898, 3.03376, 3.8, 0.06617],
                        [5.33906, 3.06133, 3.8, 0.06617],
                        [5.08914, 3.08896, 3.8, 0.06617],
                        [4.83923, 3.11664, 3.8, 0.06617],
                        [4.58931, 3.1443, 3.8, 0.06617],
                        [4.33949, 3.17159, 3.8, 0.06613],
                        [4.08993, 3.19802, 3.8, 0.06604],
                        [3.84075, 3.22314, 3.8, 0.0659],
                        [3.59214, 3.24642, 3.8, 0.06571],
                        [3.34427, 3.26736, 3.8, 0.06546],
                        [3.09732, 3.28543, 3.8, 0.06516],
                        [2.85153, 3.29994, 3.8, 0.06479],
                        [2.60707, 3.31046, 3.8, 0.06439],
                        [2.36411, 3.31659, 3.8, 0.06396],
                        [2.12287, 3.31773, 3.5516, 0.06793],
                        [1.8835, 3.31352, 3.27855, 0.07302],
                        [1.6461, 3.30381, 3.05503, 0.07777],
                        [1.41064, 3.28869, 2.73979, 0.08612],
                        [1.17729, 3.26781, 2.46179, 0.09517],
                        [0.94636, 3.24041, 2.17102, 0.10712],
                        [0.71824, 3.20551, 2.17102, 0.1063],
                        [0.49333, 3.1622, 2.17102, 0.1055],
                        [0.2724, 3.10866, 2.17102, 0.10471],
                        [0.05635, 3.04274, 2.17102, 0.10404],
                        [-0.15337, 2.96118, 2.60619, 0.08634],
                        [-0.3587, 2.86884, 3.05043, 0.07381],
                        [-0.56084, 2.76857, 3.62609, 0.06223],
                        [-0.76075, 2.66263, 3.8, 0.05954],
                        [-0.9592, 2.55289, 3.8, 0.05968],
                        [-1.15698, 2.44106, 3.64829, 0.06228],
                        [-1.35871, 2.3301, 3.4148, 0.06742],
                        [-1.56191, 2.22213, 3.21159, 0.07165],
                        [-1.76683, 2.118, 3.10756, 0.07397],
                        [-1.97379, 2.01863, 3.06081, 0.075],
                        [-2.18307, 1.92499, 3.05969, 0.07494],
                        [-2.39493, 1.8379, 3.05969, 0.07486],
                        [-2.60957, 1.75825, 3.05969, 0.07483],
                        [-2.82708, 1.68658, 3.05969, 0.07485],
                        [-3.04744, 1.62319, 3.05969, 0.07494],
                        [-3.27057, 1.56816, 3.1034, 0.07405],
                        [-3.49633, 1.52134, 3.19659, 0.07213],
                        [-3.7245, 1.48237, 3.34829, 0.06913],
                        [-3.95482, 1.45066, 3.57099, 0.06511],
                        [-4.18698, 1.42542, 3.8, 0.06145],
                        [-4.42056, 1.40575, 3.8, 0.06169],
                        [-4.65509, 1.3907, 3.8, 0.06184],
                        [-4.88992, 1.37935, 3.70349, 0.06348],
                        [-5.12444, 1.37071, 2.98492, 0.07862],
                        [-5.35845, 1.36357, 2.70669, 0.0865],
                        [-5.58771, 1.35623, 2.30919, 0.09933],
                        [-5.81557, 1.34581, 2.02986, 0.11237],
                        [-6.04083, 1.32967, 1.78139, 0.12678],
                        [-6.26221, 1.30502, 1.53787, 0.14484],
                        [-6.47901, 1.27048, 1.53787, 0.14275],
                        [-6.68959, 1.22299, 1.53787, 0.14037],
                        [-6.89209, 1.1597, 1.53787, 0.13796],
                        [-7.08376, 1.07718, 1.53787, 0.13569],
                        [-7.25966, 0.97092, 1.7825, 0.11529],
                        [-7.42269, 0.8485, 1.87734, 0.1086],
                        [-7.57332, 0.71255, 1.82442, 0.11122],
                        [-7.71143, 0.56474, 1.71609, 0.11788],
                        [-7.83632, 0.40628, 1.5696, 0.12855],
                        [-7.94721, 0.23847, 1.43333, 0.14032],
                        [-8.04297, 0.06263, 1.29226, 0.15494],
                        [-8.12131, -0.12006, 1.15355, 0.17232],
                        [-8.17941, -0.30777, 1.0, 0.19649],
                        [-8.21332, -0.49771, 1.0, 0.19294],
                        [-8.21929, -0.6856, 1.0, 0.18799],
                        [-8.19366, -0.86562, 1.0, 0.18183],
                        [-8.13324, -1.03037, 1.0, 0.17548],
                        [-8.03385, -1.16887, 1.03894, 0.16408],
                        [-7.90397, -1.27857, 1.15189, 0.14759],
                        [-7.75178, -1.3615, 1.28754, 0.13462],
                        [-7.58269, -1.41996, 1.44994, 0.12339],
                        [-7.40071, -1.45635, 1.65797, 0.11193],
                        [-7.20908, -1.47362, 1.88474, 0.10209],
                        [-7.01008, -1.47439, 2.20766, 0.09014],
                        [-6.80575, -1.46194, 2.73267, 0.07491],
                        [-6.59795, -1.44022, 2.57245, 0.08122],
                        [-6.38846, -1.4138, 2.2416, 0.09419],
                        [-6.17935, -1.38987, 1.93345, 0.10886],
                        [-5.97093, -1.37051, 1.70713, 0.12261],
                        [-5.76365, -1.35852, 1.68204, 0.12344],
                        [-5.558, -1.35666, 1.68204, 0.12227],
                        [-5.35454, -1.36779, 1.68204, 0.12114],
                        [-5.15412, -1.39593, 1.68204, 0.12032],
                        [-4.95793, -1.4456, 1.68204, 0.12032],
                        [-4.76661, -1.5177, 1.93212, 0.10582],
                        [-4.57938, -1.60735, 2.30405, 0.0901],
                        [-4.39526, -1.70988, 2.88033, 0.07317],
                        [-4.21325, -1.82098, 3.8, 0.05612],
                        [-4.03225, -1.93615, 3.8, 0.05646],
                        [-3.839, -2.05701, 3.8, 0.05998],
                        [-3.64377, -2.17631, 3.70746, 0.06171],
                        [-3.44613, -2.29336, 3.55095, 0.06469],
                        [-3.24591, -2.4076, 3.45215, 0.06678],
                        [-3.04281, -2.51824, 3.39567, 0.06811],
                        [-2.83659, -2.62449, 3.3749, 0.06874],
                        [-2.62708, -2.72552, 3.3749, 0.06892],
                        [-2.41421, -2.82071, 3.3749, 0.06909],
                        [-2.198, -2.90956, 3.3749, 0.06926],
                        [-1.97852, -2.99172, 3.3749, 0.06944],
                        [-1.7559, -3.06698, 3.38538, 0.06942],
                        [-1.53035, -3.13525, 3.42271, 0.06885],
                        [-1.30218, -3.19656, 3.48192, 0.06786],
                        [-1.07171, -3.25101, 3.55706, 0.06658],
                        [-0.83935, -3.29878, 3.6409, 0.06515],
                        [-0.60555, -3.34006, 3.72493, 0.06374],
                        [-0.37077, -3.37508, 3.79964, 0.06248],
                        [-0.13544, -3.40403, 3.77693, 0.06277],
                        [0.09997, -3.42706, 3.68781, 0.06414],
                        [0.33507, -3.44427, 3.58495, 0.06576],
                        [0.5695, -3.45569, 3.47722, 0.0675],
                        [0.80297, -3.46127, 3.37229, 0.06925],
                        [1.03523, -3.4609, 3.2761, 0.0709],
                        [1.26612, -3.4544, 3.19285, 0.07234],
                        [1.4955, -3.44149, 3.12522, 0.07351],
                        [1.7233, -3.4219, 3.07477, 0.07436],
                        [1.94946, -3.39529, 3.0423, 0.07485],
                        [2.17393, -3.36132, 3.02813, 0.07497],
                        [2.39669, -3.31964, 3.02813, 0.07484],
                        [2.61768, -3.26994, 3.02813, 0.0748],
                        [2.83685, -3.21196, 3.02813, 0.07487],
                        [3.05412, -3.14548, 3.02813, 0.07503],
                        [3.26939, -3.07037, 3.03233, 0.07519],
                        [3.48256, -2.98657, 3.05488, 0.07498],
                        [3.69351, -2.89414, 3.09577, 0.0744],
                        [3.90214, -2.79321, 3.15513, 0.07345],
                        [4.10833, -2.68399, 3.23326, 0.07217],
                        [4.31202, -2.5668, 3.33075, 0.07055],
                        [4.51314, -2.44202, 3.4486, 0.06863],
                        [4.71167, -2.3101, 3.58847, 0.06642],
                        [4.90762, -2.17154, 3.75302, 0.06395],
                        [5.10103, -2.02687, 3.8, 0.06356],
                        [5.292, -1.87666, 3.8, 0.06394],
                        [5.48064, -1.72148, 3.8, 0.06428],
                        [5.6671, -1.56191, 3.8, 0.06458],
                        [5.85154, -1.39851, 3.8, 0.06485],
                        [6.03416, -1.23181, 3.8, 0.06507],
                        [6.21515, -1.06232, 3.8, 0.06525],
                        [6.39453, -0.89019, 3.8, 0.06542],
                        [6.57232, -0.71561, 3.8, 0.06557],
                        [6.74845, -0.53863, 3.8, 0.06571],
                        [6.92279, -0.35921, 3.31793, 0.0754],
                        [7.095, -0.17714, 2.61878, 0.0957],
                        [7.26466, 0.00785, 2.21392, 0.11338],
                        [7.43106, 0.19616, 1.92877, 0.13029],
                        [7.5927, 0.38806, 1.71058, 0.14668],
                        [7.74523, 0.5827, 1.55023, 0.15952],
                        [7.88247, 0.7794, 1.42609, 0.16818],
                        [7.99934, 0.97687, 1.30552, 0.17576],
                        [8.09201, 1.1734, 1.20205, 0.18076],
                        [8.1579, 1.36705, 1.10482, 0.18515],
                        [8.19595, 1.55594, 1.10482, 0.1744],
                        [8.2059, 1.73833, 1.10482, 0.16534],
                        [8.18678, 1.91228, 1.10482, 0.1584],
                        [8.13693, 2.07541, 1.10482, 0.15439],
                        [8.05279, 2.22396, 1.20884, 0.14123],
                        [7.93971, 2.35805, 1.32288, 0.1326],
                        [7.80074, 2.4775, 1.47387, 0.12433],
                        [7.63874, 2.5824, 1.63729, 0.11788],
                        [7.45597, 2.67272, 1.86521, 0.1093],
                        [7.25555, 2.7493, 2.13325, 0.10058],
                        [7.04052, 2.81339, 2.42132, 0.09267]]

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
        STANDARD_TIME = 25  # seconds (time that is easily done by model)
        FASTEST_TIME = 17  # seconds (best time of 1st place on the track)
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
