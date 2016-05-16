#!/usr/bin/env python
import rospy
import math
import itertools
from copy import deepcopy
from std_msgs.msg import Bool, String, Float32, Float32MultiArray
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
from cse_190_assi_1.srv import requestMapData, requestTexture, moveService
from read_config import read_config

class Robot:
    def __init__(self):
        self.config = read_config()
        rospy.init_node('robot', anonymous=True)

        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture
        )

        self.make_move = rospy.ServiceProxy(
                "moveService",
                moveService
        )

        self.temperature_subscriber = rospy.Subscriber(
                "/temp_sensor/data",
                temperatureMessage,
                self.handle_temperature_message
        )

        self.activation_publisher = rospy.Publisher(
                '/temp_sensor/activation',
                Bool,
                queue_size = 10
        )
        
        self.probability_publisher = rospy.Publisher(
                '/results/probabilities',
                RobotProbabilities,
                queue_size = 10
        )
        
        self.texture_publisher = rospy.Publisher(
                '/results/texture_data',
                String,
                queue_size = 10
        )

        self.temperature_publisher = rospy.Publisher(
                '/results/temperature_data',
                Float32,
                queue_size = 10
        )

        self.complete_publisher = rospy.Publisher(
                '/map_node/sim_complete',
                Bool,
                queue_size = 10
        )

        self.temp_dict = {
                'H': 40.0,
                'C': 20.0,
                '-': 25.0
        }

        self.move_list = self.config['move_list']
        self.move_count = 0

        self.map_rows = len(self.config['pipe_map'])
        self.map_cols = len(self.config['pipe_map'][0])

        # Belief array
        self.belief = []
        for x in xrange(4):
            temp = []
            for y in xrange(5):
                temp.append(0.05)
            self.belief.append(temp)

        self.rate = rospy.Rate(1)
        rospy.sleep(1) 
        self.activation_publisher.publish(True)
        rospy.spin()
    
    #Callback function for the temperature subscriber.
    def handle_temperature_message(self, message):
        #print "incoming message, ", message
        temperature_reading = message.temperature
        print 'Temperature: ', temperature_reading
        # Bayes update based on temperature
        self.bayesfilter_temperature(temperature_reading)

        texture_response = self.texture_requester()
        texture_reading = texture_response.data
        print 'Texture: ', texture_reading
        # Bayes update based on texture
        self.bayesfilter_texture(texture_reading)

        # Completed, no more moves!
        if self.move_count == len(self.move_list):
            # Final temperature and texture reading and publishes
            self.temperature_publisher.publish(temperature_reading)
            self.texture_publisher.publish(texture_reading)
            self.probability_publisher.publish(list(itertools.chain.from_iterable(self.belief)))
            self.complete_publisher.publish(True)
            rospy.sleep(1) 
            rospy.signal_shutdown(self)

        # Make a move
        move = self.move_list[self.move_count % len(self.move_list)]
        print 'Move: ', move
        self.make_move(move)
        self.move_count += 1
        # Update(not bayes rule) based on move
        self.total_prob_motion(move)

        # Publish updated beliefs after robot finishes sensing and moving
        self.temperature_publisher.publish(temperature_reading)
        self.texture_publisher.publish(texture_reading)
        self.probability_publisher.publish(list(itertools.chain.from_iterable(self.belief)))

    def bayesfilter_texture(self, texture_reading):
        p_correct = self.config['prob_tex_correct']
        p_incorrect = 1 - p_correct
        texture_map = self.config['texture_map'] 
        for x in xrange(len(self.belief)):
            for y in xrange(len(self.belief[x])):
                if texture_reading == texture_map[x][y]:
                    # prob that texture sensor is correct
                    self.belief[x][y] = p_correct * self.belief[x][y]
                else:
                    # prob that texture sensor is wrong
                    self.belief[x][y] = p_incorrect * self.belief[x][y]

        # Find k by adding up non-normalized beliefs
        k = sum(sum(x) for x in self.belief)

        # Normalize with k
        for x in xrange(len(self.belief)):
            for y in xrange(len(self.belief[x])):
                self.belief[x][y] = self.belief[x][y] / k

    def bayesfilter_temperature(self, temperature_reading):
        pipe_map = self.config['pipe_map']
        std = self.config['temp_noise_std_dev']    
        t = temperature_reading
        mean = 0
        for x in xrange(len(self.belief)):
            for y in xrange(len(self.belief[x])):
                mean = self.temp_dict[pipe_map[x][y]]
                self.belief[x][y] = self.gaussian(t, mean, std) * self.belief[x][y]

        # Find k by adding up non-normalized beliefs
        k = sum(sum(x) for x in self.belief)

        # Normalize with k
        for x in xrange(len(self.belief)):
            for y in xrange(len(self.belief[x])):
                self.belief[x][y] = self.belief[x][y] / k

    # Gaussian random variable
    def gaussian(self, t, mean, std):
        coefficient = 1.0 / (std * math.sqrt(2 * math.pi))
        exponent = -math.pow(t - mean, 2) / (2 * math.pow(std, 2))
        val = coefficient * math.exp(exponent)
        #return (1.0 / (std * math.sqrt(2 * math.pi))) * math.exp( (-(t - mean)**2)/(2*std**2))
        return val

    def total_prob_motion(self, move):
        possible_moves = self.config['possible_moves']
        p_correct = self.config['prob_move_correct']
        p_incorrect = 1 - p_correct
        belief = deepcopy(self.belief)
        for x in xrange(len(self.belief)):
            for y in xrange(len(self.belief[x])):
                psum = 0
                # Correct move
                psum += p_correct * self.belief[(x - move[0]) % self.map_rows][(y - move[1]) % self.map_cols]
                # Sum up incorrect moves
                opposite_move = [-move[0], -move[1]]
                for pm in possible_moves:
                    if pm != opposite_move:
                        temp_vertical = (x + pm[0]) % self.map_rows
                        temp_horizontal = (y + pm[1]) % self.map_cols
                        psum += (p_incorrect / 4) * self.belief[temp_vertical][temp_horizontal]
                belief[x][y] = psum
        # Set global belief table to newly calculated one
        self.belief = belief

if __name__ == '__main__':
    try:
        robot = Robot()
    except rospy.ROSInterruptException:
        pass
