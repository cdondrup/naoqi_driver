import rospy
from service_abstractclass import AbstractService
from nao_interaction_msgs.srv import SetBreathEnabled, SetBreathEnabledResponse
from nao_interaction_msgs.srv import GoToPose, GoToPoseResponse
from std_srvs.srv import Empty, EmptyResponse
import utils as ut
import tf


class MotionServices(AbstractService):
    def __init__(self, super_ns):
        super(MotionServices, self).__init__(
            proxy_name="ALMotion",
            ns=super_ns+"/motion",
            topics=["move_to", "rest", "set_breath_enabled", "wake_up"],
            service_types=[GoToPose, Empty, SetBreathEnabled, Empty])
        self.listener = tf.TransformListener()

    def transform(self, pose_stamped, target_frame):
        if pose_stamped.header.frame_id != target_frame:
            try:
                t = self.listener.getLatestCommonTime(target_frame, pose_stamped.header.frame_id)
                pose_stamped.header.stamp = t
                return self.listener.transformPose(target_frame, pose_stamped)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
                return None
        else:
            return pose_stamped

    def move_to_callback(self, req):
        req.pose = self.transform(req.pose, "base_footprint")

        yaw = ut.get_yaw(req.pose.pose)

        rospy.loginfo("going to move x: {x} y: {y} z: {z} yaw: {yaw}".format(
            x=req.pose.pose.position.x,
            y=req.pose.pose.position.y,
            z=req.pose.pose.position.z,
            yaw=yaw))
        self.proxy.moveTo(req.pose.pose.position.x, req.pose.pose.position.y, yaw)

        return GoToPoseResponse()

    def rest_callback(self, req):
        self.proxy.rest()
        return EmptyResponse()

    def set_breath_enabled_callback(self, req):
        if req.chain_name in (req.HEAD, req.BODY, req.ARMS, req.LEGS, req.LARM, req.RARM):
            self.proxy.setBreathEnabled(req.chain_name, req.enable)
        else:
            rospy.logerr("Unknown body part")
        return SetBreathEnabledResponse()

    def wake_up_callback(self, req):
        self.proxy.wakeUp()
        return EmptyResponse()
