import rospy
from sensor_msgs.msg import PointCloud2
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from pcdarray import PC2_Array


def callback(data):
    points = PC2_Array(data)
    rospy.loginfo('Publish PCD to /pointcloud_array Successful')
    pub.publish(points.flatten())


def test():
    rospy.init_node('test_pub_node')

    rospy.Subscriber(
        '/velodyne_points', 
        PointCloud2, 
        callback=callback
    )
    
    rospy.spin()


if __name__ == '__main__':
    try:   
        pub = rospy.Publisher(
            '/pointcloud_array', 
            numpy_msg(Floats), 
            queue_size=1
        )
        test()
    except rospy.ROSInterruptException:
        pass