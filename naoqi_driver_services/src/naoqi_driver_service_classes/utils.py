from tf import transformations


def get_yaw(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    return transformations.euler_from_quaternion(quaternion)[2]
