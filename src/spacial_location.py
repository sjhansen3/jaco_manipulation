import autolab_core
import numpy as np
import geometry_msgs.msg

class Quaternion:
    def __init__(self, orientation, enforce_norm=True):
        self._orientation = orientation
        self._enforce_norm = enforce_norm
        self._check_valid()


    def _check_valid(self):
        """
        Params
        Raises
        ---
        ValueError
            if the data is not (4,) numpy array
            if quaternion does not have norm 1
        """
        if len(self.orientation) != 4:
            raise ValueError('Quaternions must be (4,) array like object') 
        if self._enforce_norm:
            norm = self.norm
            if not np.allclose(norm,[1]):
                raise ValueError("quaternion does not have norm 1. Got {0}".format(norm))
    
    @property
    def norm(self):
        return np.linalg.norm(self.orientation)

    @property
    def orientation(self):
        return self._orientation
    @property
    def x(self):
        return self._orientation[0]

    @property 
    def y(self):
        return self._orientation[1]

    @property
    def z(self):
        return self._orientation[2]

    @property
    def w(self):
        return self._orientation[3]

    def __getitem__(self, i):
        """Return a single element from the collection.
        Parameters
        ----------
        i : indexing-type (int or slice or list)
            The index of the desired element.
        Returns
        -------
        :obj:`Point` or :obj:`PointCloud`
            The returned element or group.
        """
        if isinstance(i, int):
            if i >= 4:
                raise ValueError('Index %d is out of bounds' %(i))
            return self.orientation[i]
        if isinstance(i, list):
            i = np.array(i)
        if isinstance(i, np.ndarray):
            if np.max(i) >= 4:
                raise ValueError('Index %d is out of bounds' %(np.max(i)))
            return self.orientation[i]
        if isinstance(i, slice):
            return self.orientation[i]
        raise ValueError('Type %s not supported for indexing' %(type(i)))

    def __str__(self):
        return str(self._orientation)

    def __repr__(self):
        return str(self._orientation)

    @property
    def ros_message(self):
        q = geometry_msgs.msg.Quaternion()
        q.w = self.w
        q.x = self.x
        q.y = self.y
        q.z = self.z
        return q

class Pose:
    """ class for maintaining pose of an object in space
    """
    def __init__(self, position, orientation, frame="unspecified"):
        if isinstance(orientation, Quaternion):
            o = orientation
        else:
            o = Quaternion(orientation)
        if isinstance(position, autolab_core.Point):
            p = position
        else:
            p = autolab_core.Point(np.asarray(position), frame)
        self._point = p
        self._orientation = o
    
    @property
    def ros_message(self):
        point = geometry_msgs.msg.Point()
        point.x = self._point.x
        point.y = self._point.y
        point.z = self._point.z

        pose = geometry_msgs.msg.Pose()
        pose.orientation = self._orientation.ros_message
        pose.position = point
        return pose

    @property
    def position(self):
        return self._point

    @property
    def orientation(self):
        return self._orientation

    def __str__(self):
        return str(self._point)+str(self._orientation)

    def __repr__(self):
        return "pos: {}, orientatino: {}".format(self._point, self._orientation)

if __name__ == "__main__":
    #test some of the functionality
    print(Pose( np.asarray([1,2,3]), [0,0,0,1]) )
    pose = Pose( np.asarray([1,2,3]), [0,0,0,1])
    print(pose.position.x)
    print(pose.position.y)
    print(pose.position.z)
    print(pose.orientation.x)
    print(pose.orientation[1])
    print(pose.ros_message)
