import rospy
import yaml
from yaml.loader import SafeLoader

if __name__=="__main__":

    rospy.init_node("autonomous_node")
    obstacles_file = rospy.get_param("obstacles_file")

    with open(obstacles_file) as f:
        obs_locations = yaml.load(f, Loader=SafeLoader) 
    
    
