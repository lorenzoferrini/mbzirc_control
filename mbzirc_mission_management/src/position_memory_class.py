import os.path
from geopy.distance import geodesic
from math import sqrt
from ConfigParser import SafeConfigParser

class Position_Memory:
    def __init__(self):
	path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "config/configuration.ini")
        config = SafeConfigParser()
        config.read(path)
        self.__max_distance= config.getfloat('position_memory','max_distance')
        self.__list_balloon_already_reached = []   #list of balloon already reached

    # create new object
    # def new_ballon_position(self, latitude, longitude, altitude):                   
    #     return { "position": (latitude, longitude), "altitude": altitude}

    # given two points returns the distance in meters
    def __balloon_distance(self, latitude, longitude, altitude, old_balloon):
        flat_distance = geodesic((latitude,longitude), old_balloon['position']).meters
        altitude_difference = abs(altitude - old_balloon['altitude'])
        return sqrt(flat_distance*flat_distance+altitude_difference*altitude_difference)

    # main function returns true if the given point has already been visited previously false otherwise
    def check_ballon_already_reached(self, latitude, longitude, altitude):
        print '##########',str(latitude),str(longitude), str(altitude)
        print self.__list_balloon_already_reached
        for balloon in self.__list_balloon_already_reached:
            if self.__balloon_distance(latitude,longitude,altitude,balloon) <= self.__max_distance:
                #found_flag=True
                #break
                print '------------true'
                return True
        print '-------------false'
        return False

    # Add to the list the balloon
    def balloon_reached(self, latitude, longitude, altitude):
        self.__list_balloon_already_reached.append({ "position": (latitude, longitude), "altitude": altitude})

    def wipe_memory(self):
        self.__list_balloon_already_reached = []
