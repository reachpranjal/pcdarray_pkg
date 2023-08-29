# PointCloud2 to Numpy Array

A goto package to convert `PointCloud2` message to `numpy` array that just works !

## Disclaimer
- Tested only on Velodyne LiDAR VLP-16.
- Returns only X, Y, Z points in `np.float32` format

## How to install

(Optional) To run inside conda environment :
```
$ conda create -n pcdarray_env python=3.10
$ conda activate pcdarray_env
```
Install the package
```
$ pip install git+https://github.com/reachpranjal/pcdarray_pkg.git
```

## How to use

### 1. Converting `PointCloud2` message
```python
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
```

### 2. Publishing to a new topic

- Publisher

```python
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
```

- Subscriber

```python
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
```