import os.path
from math import sqrt
from collections import deque
from ConfigParser import SafeConfigParser

# class to simulate the memory in the vision system
#
# This object helps the vision system to not lose the balloon even when the recognition is not successful or
#  in the case of fakes positives appear for a few frames

class Memory_Frame:
    def __init__(self):
        path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "config/configuration.ini")
        self.config = SafeConfigParser()
        self.config.read(path)
        # Minimum number of frames required before accepting a balloon
        self.min_num_frame_memory = self.config.getint('memory_class', 'min_num_frame_memory')
        self.max_distance = self.config.getint('memory_class', 'max_distance')
        self.list_history = []

    def limit(self,num):
        minimum = self.config.getint('memory_class', 'minimum')
        maximum = self.config.getint('memory_class', 'maximum')
        return max(min(num, maximum), minimum)

    # New memory for new balloon
    def new_history(self, x, y, radius):
        queue_ballon = deque(maxlen=self.config.getint('memory_class', 'queue_size'))
        queue_ballon.append([x, y, radius])
        self.list_history.append([1, queue_ballon])

    def ballon_insert(self,x,y,radius):

        if len(self.list_history)==0:
            self.new_history(x, y, radius)
        else:
            min_distance = 999999
            index_min_distance = -1
            for index, one_history in enumerate(self.list_history):
                one_history[0]=self.limit(one_history[0]-1)
                last_balloon = one_history[1][-1]
                distance = sqrt((int(last_balloon[0]) - x) ** 2 + (int(last_balloon[1]) - y) ** 2)
                if distance < min_distance and distance < self.max_distance:
                    min_distance = distance
                    index_min_distance = index
            if index_min_distance >= 0:
                self.list_history[index_min_distance][0] = self.limit(self.list_history[index_min_distance][0]+2)
                self.list_history[index_min_distance][1].append([x,y,radius])
            else:
                self.new_history(x,y,radius)
            self.list_history = list(filter(lambda x: x[0] != 0, self.list_history))
            self.list_history.sort(key=lambda r: r[0], reverse=True)
            if self.list_history[0][0] >= self.min_num_frame_memory:
                return self.list_history[0][1][-1]
            else:
                return []


    def last_balloon(self):
        for index, one_history in enumerate(self.list_history):
            one_history[0] = self.limit(one_history[0] - 1)
        self.list_history = list(filter(lambda x: x[0] != 0, self.list_history))
        if len(self.list_history) > 0:
            if self.list_history[0][0] >= self.min_num_frame_memory:
                return self.list_history[0][1][-1]
            else:
                return []
        else:
            return []






