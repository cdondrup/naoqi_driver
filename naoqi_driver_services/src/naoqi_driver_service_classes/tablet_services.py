import rospy
from service_abstractclass import AbstractService
from std_srvs.srv import Empty, EmptyResponse
from nao_interaction_msgs.srv import String, StringResponse


class TabletServices(AbstractService):
    def __init__(self, super_ns):
        super(TabletServices, self).__init__(
            proxy_name="ALTabletService",
            ns=super_ns+"/tablet",
            topics=["show_webview", "load_url", "execute_js"],
            service_types=[Empty, String, String])

    def show_webview_callback(self, req):
        # print "show webview"
        self.proxy.showWebview()
        return EmptyResponse()

    def load_url_callback(self, req):
        # print "load url", req.request
        self.proxy.loadUrl(req.request)
        return StringResponse()

    def execute_js_callback(self, req):
        # print "execute js", req.request
        self.proxy.executeJS(req.request)
        return StringResponse()

