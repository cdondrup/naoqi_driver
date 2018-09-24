import rospy
from abc import ABCMeta, abstractmethod
from naoqi_interface_classes.service_proxy import ServiceProxy


class AbstractService(object):
    __metaclass__ = ABCMeta

    def __init__(self, proxy_name, ns, topics, service_types):
        rospy.loginfo("Starting %s services" % proxy_name)
        self.proxy = ServiceProxy(proxy_name=proxy_name)
        if not isinstance(topics, list) or not isinstance(service_types, list):
            raise TypeError("Topics and service types have to be lists of equal length")
        self.srvs = [rospy.Service(ns+"/"+topic, service_type, getattr(self, topic+"_callback")) for topic, service_type in zip(topics, service_types)]
