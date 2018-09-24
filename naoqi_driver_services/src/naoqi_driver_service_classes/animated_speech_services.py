from service_abstractclass import AbstractService
from nao_interaction_msgs.srv import Say, SayResponse


class AnimatedSpeechServices(AbstractService):
    def __init__(self, super_ns):
        super(AnimatedSpeechServices, self).__init__(
            proxy_name="ALAnimatedSpeech",
            ns=super_ns+"/animated_speech",
            topics=["say"],
            service_types=[Say])

    def say_callback(self, req):
        self.proxy.say(req.text)
        return SayResponse()
