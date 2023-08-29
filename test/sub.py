import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


def callback(data):
    points = data.data.reshape(-1, 3)
    rospy.loginfo(f'PCD Shape: {points.shape}')


def test():
    rospy.init_node('test_sub_node')
    
    rospy.Subscriber(
        '/pointcloud_array', 
        numpy_msg(Floats), 
        callback=callback
    )
    rospy.spin()


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
