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
        racing_track = [[6.83743, 2.89489, 3.73778, 0.06566],
[6.58874, 2.92402, 4.0, 0.0626],
[6.3388, 2.95137, 4.0, 0.06286],
[6.08885, 2.97878, 4.0, 0.06286],
[5.83892, 3.00624, 4.0, 0.06286],
[5.58898, 3.03376, 4.0, 0.06286],
[5.33906, 3.06133, 4.0, 0.06286],
[5.08914, 3.08896, 4.0, 0.06286],
[4.83923, 3.11664, 4.0, 0.06286],
[4.58931, 3.1443, 4.0, 0.06286],
[4.33939, 3.17195, 4.0, 0.06286],
[4.08948, 3.19958, 4.0, 0.06286],
[3.83955, 3.22719, 4.0, 0.06286],
[3.58963, 3.25479, 4.0, 0.06286],
[3.33971, 3.28237, 4.0, 0.06286],
[3.08978, 3.30994, 4.0, 0.06286],
[2.84023, 3.33626, 3.87891, 0.06469],
[2.59141, 3.36005, 3.61628, 0.06912],
[2.34371, 3.38009, 3.39745, 0.07315],
[2.09752, 3.39519, 3.20955, 0.07685],
[1.85324, 3.40424, 3.04073, 0.08039],
[1.61126, 3.40621, 2.89154, 0.08369],
[1.37199, 3.4001, 2.75547, 0.08686],
[1.13582, 3.385, 2.63029, 0.08997],
[0.90317, 3.36007, 2.33937, 0.10002],
[0.67446, 3.3245, 2.33937, 0.09894],
[0.45015, 3.27746, 2.33937, 0.09797],
[0.23073, 3.21811, 2.33937, 0.09716],
[0.01679, 3.14555, 2.33937, 0.09657],
[-0.191, 3.05877, 2.33937, 0.09626],
[-0.39066, 2.95425, 2.70505, 0.08331],
[-0.58394, 2.83648, 3.31623, 0.06825],
[-0.77283, 2.70983, 3.97245, 0.05725],
[-0.95947, 2.57865, 3.71547, 0.0614],
[-1.15148, 2.4475, 3.48974, 0.06663],
[-1.34583, 2.31979, 3.30007, 0.07047],
[-1.54285, 2.19649, 3.13304, 0.07418],
[-1.74285, 2.0786, 2.97919, 0.07792],
[-1.94606, 1.96714, 2.97919, 0.0778],
[-2.15271, 1.8631, 2.97919, 0.07766],
[-2.36301, 1.76756, 2.97919, 0.07753],
[-2.57714, 1.68158, 2.97919, 0.07745],
[-2.79525, 1.60629, 2.97919, 0.07745],
[-3.01752, 1.54296, 3.16519, 0.07302],
[-3.24325, 1.49035, 3.29812, 0.07028],
[-3.47206, 1.44775, 3.44374, 0.06758],
[-3.70357, 1.41451, 3.60353, 0.06491],
[-3.93748, 1.38994, 3.78346, 0.06216],
[-4.17351, 1.37341, 3.98998, 0.0593],
[-4.41138, 1.36423, 4.0, 0.05951],
[-4.65084, 1.36171, 3.59325, 0.06665],
[-4.89166, 1.36514, 3.24359, 0.07425],
[-5.13362, 1.37382, 2.97943, 0.08126],
[-5.36974, 1.38663, 2.7667, 0.08547],
[-5.60429, 1.39436, 2.59007, 0.09061],
[-5.83664, 1.39517, 2.43753, 0.09532],
[-6.06616, 1.38732, 2.30655, 0.09957],
[-6.29221, 1.36923, 2.24795, 0.10088],
[-6.51416, 1.33947, 2.17532, 0.10294],
[-6.73129, 1.2967, 2.10616, 0.10508],
[-6.94285, 1.23961, 2.03949, 0.10744],
[-7.14792, 1.16685, 1.97831, 0.10999],
[-7.34542, 1.07702, 1.91386, 0.11337],
[-7.53442, 0.96945, 1.85341, 0.11734],
[-7.71361, 0.84299, 1.79547, 0.12215],
[-7.88117, 0.69618, 1.7381, 0.12818],
[-8.03461, 0.52718, 1.65582, 0.13785],
[-8.17027, 0.33401, 1.57748, 0.14964],
[-8.28225, 0.11564, 1.50849, 0.16268],
[-8.35909, -0.11259, 1.44257, 0.16694],
[-8.39782, -0.33515, 1.36835, 0.16509],
[-8.40194, -0.54772, 1.36835, 0.15537],
[-8.37384, -0.74744, 1.36835, 0.14739],
[-8.31546, -0.93164, 1.36835, 0.14122],
[-8.22845, -1.0975, 1.36835, 0.13688],
[-8.1141, -1.24152, 1.36835, 0.1344],
[-7.97321, -1.35853, 1.44958, 0.12634],
[-7.81159, -1.4486, 1.54564, 0.11971],
[-7.63356, -1.51229, 1.66321, 0.11368],
[-7.44254, -1.55048, 1.80787, 0.10775],
[-7.24137, -1.56452, 1.99595, 0.10104],
[-7.0325, -1.55639, 2.25644, 0.09263],
[-6.81814, -1.52907, 2.51482, 0.08593],
[-6.60017, -1.48658, 2.17834, 0.10195],
[-6.38018, -1.43404, 1.8123, 0.1248],
[-6.16973, -1.38001, 1.8123, 0.11989],
[-5.95964, -1.33165, 1.8123, 0.11896],
[-5.75027, -1.29391, 1.8123, 0.11739],
[-5.54211, -1.27133, 1.8123, 0.11553],
[-5.33585, -1.26806, 1.8123, 0.11383],
[-5.13311, -1.29162, 1.8168, 0.11234],
[-4.9348, -1.34028, 1.8168, 0.11239],
[-4.74238, -1.41527, 2.03416, 0.10152],
[-4.55572, -1.51189, 2.35092, 0.08941],
[-4.37398, -1.62544, 2.87883, 0.07444],
[-4.19584, -1.75084, 4.0, 0.05446],
[-4.01963, -1.88243, 4.0, 0.05498],
[-3.83206, -2.01909, 3.99482, 0.05809],
[-3.64205, -2.15306, 3.81559, 0.06093],
[-3.44928, -2.28367, 3.64571, 0.06387],
[-3.25345, -2.4102, 3.64571, 0.06395],
[-3.05431, -2.53193, 3.64571, 0.06402],
[-2.85166, -2.64821, 3.64571, 0.06409],
[-2.64528, -2.75831, 3.64571, 0.06416],
[-2.43499, -2.86152, 3.64571, 0.06426],
[-2.22053, -2.95707, 3.69539, 0.06353],
[-2.00212, -3.04503, 3.74273, 0.06291],
[-1.77992, -3.1255, 3.78608, 0.06242],
[-1.55405, -3.19852, 3.83291, 0.06193],
[-1.32462, -3.26414, 3.8944, 0.06127],
[-1.09178, -3.32245, 3.97398, 0.0604],
[-0.85566, -3.37358, 4.0, 0.0604],
[-0.61646, -3.41774, 4.0, 0.06081],
[-0.37438, -3.45516, 4.0, 0.06124],
[-0.12962, -3.4861, 4.0, 0.06168],
[0.11759, -3.51082, 4.0, 0.06211],
[0.36694, -3.52958, 4.0, 0.06251],
[0.61319, -3.54244, 3.90552, 0.06314],
[0.84617, -3.54976, 3.79142, 0.06148],
[1.07428, -3.55184, 3.70095, 0.06164],
[1.29974, -3.5484, 3.62384, 0.06222],
[1.52367, -3.53906, 3.5646, 0.06288],
[1.74674, -3.5233, 3.52112, 0.06351],
[1.96934, -3.50066, 3.49291, 0.06406],
[2.19164, -3.47066, 3.48725, 0.06432],
[2.41367, -3.43284, 3.48227, 0.06468],
[2.63536, -3.38678, 3.48227, 0.06502],
[2.8565, -3.33207, 3.48227, 0.06542],
[3.07685, -3.2684, 3.48227, 0.06586],
[3.29606, -3.19551, 3.48227, 0.06634],
[3.5138, -3.11327, 3.48227, 0.06684],
[3.72968, -3.02157, 3.514, 0.06675],
[3.94332, -2.92052, 3.56462, 0.0663],
[4.15442, -2.81032, 3.62721, 0.06565],
[4.36266, -2.69127, 3.70734, 0.0647],
[4.56782, -2.56373, 3.80788, 0.06344],
[4.76971, -2.42815, 3.93004, 0.06188],
[4.96821, -2.28505, 4.0, 0.06118],
[5.16323, -2.1349, 4.0, 0.06153],
[5.35473, -1.97826, 4.0, 0.06185],
[5.54274, -1.81572, 4.0, 0.06213],
[5.72734, -1.64791, 4.0, 0.06237],
[5.90864, -1.47545, 4.0, 0.06256],
[6.08685, -1.29899, 4.0, 0.0627],
[6.26219, -1.11917, 4.0, 0.06279],
[6.43494, -0.93659, 4.0, 0.06284],
[6.60542, -0.75179, 4.0, 0.06286],
[6.77401, -0.56524, 4.0, 0.06286],
[6.94111, -0.37735, 4.0, 0.06286],
[7.10682, -0.18825, 2.89589, 0.08683],
[7.27117, 0.00205, 2.33016, 0.10791],
[7.43416, 0.19351, 2.00356, 0.1255],
[7.59578, 0.38613, 1.78444, 0.14091],
[7.75597, 0.57994, 1.61882, 0.15532],
[7.90884, 0.77609, 1.49003, 0.1669],
[8.04492, 0.97473, 1.38658, 0.17365],
[8.15667, 1.17475, 1.3, 0.17625],
[8.23931, 1.37421, 1.3, 0.16608],
[8.29041, 1.57085, 1.3, 0.15629],
[8.3087, 1.76232, 1.3, 0.14795],
[8.29332, 1.94603, 1.3, 0.14181],
[8.24301, 2.11885, 1.3, 0.13846],
[8.15538, 2.27627, 1.43357, 0.12568],
[8.03731, 2.41791, 1.53447, 0.12017],
[7.89156, 2.54258, 1.65664, 0.11577],
[7.72029, 2.64914, 1.81351, 0.11123],
[7.52582, 2.73685, 2.01925, 0.10565],
[7.31107, 2.80573, 2.31219, 0.09754],
[7.07997, 2.85726, 2.78503, 0.08502]]

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
        STANDARD_TIME = 25
        FASTEST_TIME = 17
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
