from service_abstractclass import AbstractService
from nao_interaction_msgs.srv import BehaviorManagerControl, BehaviorManagerControlResponse
from nao_interaction_msgs.srv import BehaviorManagerInfo, BehaviorManagerInfoResponse


class BehaviourManagerServices(AbstractService):
    def __init__(self, super_ns):
        super(BehaviourManagerServices, self).__init__(
            proxy_name="ALBehaviorManager",
            ns=super_ns+"/behaviour_manager",
            topics=["get_installed_behaviors", "get_running_behaviors", "start_behaviour", "stop_behaviour"],
            service_types=[BehaviorManagerInfo, BehaviorManagerInfo, BehaviorManagerControl, BehaviorManagerControl])

    def get_installed_behaviors_callback(self, req):
        return BehaviorManagerInfoResponse(behaviors=self.proxy.getInstalledBehaviors())

    def get_running_behaviors_callback(self, req):
        return BehaviorManagerInfoResponse(behaviors=self.proxy.getRunningBehaviors())

    def start_behaviour_callback(self, req):
        resp = BehaviorManagerControlResponse()
        resp.success = True
        if self.proxy.isBehaviorInstalled(req.name):
            if not self.proxy.isBehaviorRunning(req.name):
                self.proxy.startBehavior(req.name)
        else:
            resp.success = False
        return resp

    def stop_behaviour_callback(self, req):
        resp = BehaviorManagerControlResponse()
        resp.success = True
        if req.name == "ALL":
            self.proxy.stopAllBehaviors()
        elif self.proxy.isBehaviorInstalled(req.name):
            if self.proxy.isBehaviorRunning(req.name):
                self.proxy.stopBehavior(req.name)
        else:
            resp.success = False
        return resp
