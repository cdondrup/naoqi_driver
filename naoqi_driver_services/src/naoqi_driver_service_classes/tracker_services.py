import rospy
from service_abstractclass import AbstractService
from std_srvs.srv import Empty, EmptyResponse
from nao_interaction_msgs.srv import SetTrackerMode, SetTrackerModeResponse
from nao_interaction_msgs.srv import SetTrackerTarget, SetTrackerTargetResponse
from nao_interaction_msgs.srv import StartTracker, StartTrackerResponse
from nao_interaction_msgs.srv import TrackerPointAt, TrackerPointAtResponse
from nao_interaction_msgs.srv import TrackerLookAt, TrackerLookAtResponse


class TrackerServices(AbstractService):
    def __init__(self, super_ns):
        super(TrackerServices, self).__init__(
            proxy_name="ALTracker",
            ns=super_ns+"/tracker",
            topics=["look_at", "point_at", "register_target", "set_mode", "stop_tracker", "track", "unregister_all_targets"],
            service_types=[TrackerLookAt, TrackerPointAt, SetTrackerTarget, SetTrackerMode, Empty, StartTracker, Empty])

    def look_at_callback(self, req):
        if req.frame in (req.FRAME_ROBOT, req.FRAME_TORSO, req.FRAME_WORLD):
            self.proxy.lookAt(
                [req.target.x, req.target.y, req.target.z],
                req.frame,
                req.max_speed_fraction,
                req.use_whole_body)
        else:
            rospy.logerr("Unknown frame '%s'" % req.frame)
        return TrackerLookAtResponse()

    def point_at_callback(self, req):
        if req.effector in (req.EFFECTOR_ARMS, req.EFFECTOR_LARM, req.EFFECTOR_RARM):
            if req.frame in (req.FRAME_ROBOT, req.FRAME_TORSO, req.FRAME_WORLD):
                self.proxy.pointAt(
                    req.effector,
                    [req.target.x, req.target.y, req.target.z],
                    req.frame,
                    req.max_speed_fraction)
            else:
                rospy.logerr("Unknown frame '%s'" % req.frame)
        else:
            rospy.logerr("Unknown effector '%s'" % req.effector)
        return TrackerPointAtResponse()

    def register_target_callback(self, req):
        if req.target == req.FACE:
            self.proxy.registerTarget(req.target, req.values[0])
        elif req.target in (req.PEOPLE, req.SOUND):
            self.proxy.registerTarget(req.target, req.values)
        else:
            rospy.logerr("Unknown target '%s'" % req.target)
        return SetTrackerTargetResponse()

    def set_mode_callback(self, req):
        if req.mode in (req.HEAD, req.WHOLEBODY, req.MOVE):
            self.proxy.setMode(req.mode)
        else:
            rospy.logerr("Unknown mode '%s'" % req.mode)
        return SetTrackerModeResponse()

    def stop_tracker_callback(self, req):
        self.proxy.stopTracker()
        return EmptyResponse()

    def track_callback(self, req):
        if req.target in (req.FACE, req.PEOPLE, req.SOUND):
            self.proxy.track(req.target)
        else:
            rospy.logerr("Unknown target '%s'" % req.target)
        return StartTrackerResponse()

    def unregister_all_targets_callback(self, req):
        self.proxy.unregisterAllTargets()
        return EmptyResponse()
