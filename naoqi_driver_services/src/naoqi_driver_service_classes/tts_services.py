from service_abstractclass import AbstractService
from nao_interaction_msgs.srv import Say, SayResponse


class TTSServices(AbstractService):
    def __init__(self, super_ns):
        super(TTSServices, self).__init__(
            proxy_name="ALTextToSpeech",
            ns=super_ns+"/tts",
            topics=["say"],
            service_types=[Say])

    def say_callback(self, req):
        self.proxy.say(req.text)
        return SayResponse()
