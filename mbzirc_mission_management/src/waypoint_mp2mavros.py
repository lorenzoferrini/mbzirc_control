import csv
import rospy
import mavros
from mavros_msgs.msg import Waypoint
import rospy
def create_waypoint_list(name):

    known_versions = (110, 120)
    csv.register_dialect('mission_planer', delimiter = '\t', doublequote=False, skipinitialspace = True,
                     lineterminator = '\r\n', quoting=csv.QUOTE_NONE)
    rospy.logwarn('WARNING')
    with  open(name, 'r') as file_wp:
        rospy.logwarn('@@@WARNING@@@')
        waypoint_list=[]
        got_header = False
        for data in csv.reader(file_wp,dialect='mission_planer'):
            if data[0].startswith('#'):
                 continue; # skip comments (i think in next format version they add this)
            if not got_header:
                 qgc, wpl, ver = data[0].split(' ', 3)
                 ver = int(ver)
                 if qgc == 'QGC' and wpl == 'WPL' and ver in known_versions:
                     got_header = True
            else:
                waypoint_list.append(Waypoint(
                    frame=int(data[2]),
                    is_current = bool(int(data[1])),
                    command = int(data[3]),
                    param1 = float(data[4]),
                    param2 = float(data[5]),
                    param3 = float(data[6]),
                    param4 = float(data[7]),
                    x_lat = float(data[8]),
                    y_long = float(data[9]),
                    z_alt = float(data[10]),
                    autocontinue = bool(int(data[11]))
                ))
    #print waypoint_list
    return waypoint_list

#if __name__ == '__main__':
    #create_waypoint_list('/home/andrea/waypoint/wp_fileMP2.waypoints')
