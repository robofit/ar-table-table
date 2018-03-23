#!/usr/bin/env python
import rospy
import jsonpickle
from art_msgs.msg import InstancesArray, ObjInstance
from std_msgs.msg import String
from art_msgs.srv import getObjectTypeRequest, getObjectTypeResponse, getObjectType


class BridgeToJsonMsg:

    def __init__(self):
        rospy.Subscriber("/art/object_detector/object_filtered", InstancesArray, self.callback)
        self.pub = rospy.Publisher("/objects_string", String, queue_size=1)
        rospy.loginfo("Waiting for /art/db/object_type/get service")
        rospy.wait_for_service("/art/db/object_type/get")
        rospy.loginfo("Service /art/db/object_type/get found")
        self.get_object_type_srv = rospy.ServiceProxy("/art/db/object_type/get", getObjectType)
        rospy.spin()

    def callback(self, detected_object):
        """
        :type detected_object: InstancesArray
        :return:
        """

        objects = []
        for obj in detected_object.instances:
            '''to_json = {'name': obj.object_id, 'pose': {'position': {'x': obj.pose.position.x,
                                                                         'y': obj.pose.position.y,
                                                                         'z': obj.pose.position.z},
                                                            'orientation': {'x': obj.pose.orientation.x,
                                                                            'y': obj.pose.orientation.y,
                                                                            'z': obj.pose.orientation.z,
                                                                            'w': obj.pose.orientation.w}}}'''
            req = getObjectTypeRequest()
            req.name = obj.object_type
            resp = self.get_object_type_srv.call(req)
            if not resp.success:
                rospy.logwarn("Object type " + str(req.name) + " not in DB, skipping")
                continue
            to_json = {'name': obj.object_id,
                       'type': obj.object_type,
                       'position': {'x': obj.pose.position.x,  #in meters 
                                    'y': obj.pose.position.y,
                                    'z': obj.pose.position.z},
                       'orientation': {'x': obj.pose.orientation.x,
                                       'y': obj.pose.orientation.y,
                                       'z': obj.pose.orientation.z,
                                       'w': obj.pose.orientation.w},
                       'bbox': {'x': resp.object_type.bbox.dimensions[0],
                                'y': resp.object_type.bbox.dimensions[1],
                                'z': resp.object_type.bbox.dimensions[2]}}
            objects.append(to_json)
        self.pub.publish(jsonpickle.encode(objects))


if __name__ == '__main__':
    rospy.init_node('bridge_to_json_msg')

    try:
        node = BridgeToJsonMsg()
    except rospy.ROSInterruptException:
        pass
