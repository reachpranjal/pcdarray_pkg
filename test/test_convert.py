import rospy
from sensor_msgs.msg import PointCloud2

from pcdarray import PC2_Array


def callback(data):
    points = PC2_Array(data)
    rospy.loginfo(f'PCD Shape: {points.shape}')


def test():
    rospy.init_node('test_node')

    rospy.Subscriber(
        '/velodyne_points', 
        PointCloud2, 
        callback=callback
    )
    rospy.spin()


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
