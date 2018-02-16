#! /usr/bin/env python
import rospy
from art_msgs.msg import InstancesArray, ObjInstance, KeyValue
from art_msgs.srv import ObjectFlagSetResponse, ObjectFlagSet, ObjectFlagClear, ObjectFlagClearResponse
from std_srvs.srv import Empty, EmptyResponse
import tf
from geometry_msgs.msg import Pose, PoseStamped
from math import sqrt
import numpy as np
from scipy.spatial import distance
import threading
from tf import transformations
from math import pi, cos, sin, atan2


# TODO publish TF

def q2a(q):
    return [q.x, q.y, q.z, q.w]


def a2q(q, arr):
    q.x = arr[0]
    q.y = arr[1]
    q.z = arr[2]
    q.w = arr[3]


class TrackedObject:
    def __init__(self, target_frame, tfl, object_id, object_type):

        self.object_id = object_id
        self.object_type = object_type
        self.tfl = tfl
        self.target_frame = target_frame
        self.max_dist = 2.0
        self.min_dist = 0.05
        self.min_meas_cnt = 5
        self.new = True
        self.lost = False

        self.meas = {}
        self.flags = {}

    def add_meas(self, ps):

        if self.lost:

            self.new = True

        self.lost = False

        dist = distance.euclidean((0, 0, 0), (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z))

        if dist > self.max_dist or dist < self.min_dist:
            rospy.logdebug("Object " + self.object_id + " seen by " + ps.header.frame_id + " is too far (or too close): " + str(dist))
            return

        try:

            pps = self.transform(ps)

        except tf.Exception as e:

            rospy.logwarn("Transform at " + str(ps.header.stamp.to_sec()) + " between " + self.target_frame +
                          " and " + ps.header.frame_id + " not available: " + str(e))
            return

        if ps.header.frame_id not in self.meas:
            self.meas[ps.header.frame_id] = []

        self.meas[ps.header.frame_id].append([dist, self.transform(pps)])

    def prune_meas(self, now, max_age):

        frames_to_delete = []

        # delete old measurements
        for frame_id, val in self.meas.iteritems():

            idx_to_delete = []

            for idx in range(0, len(val)):

                if (now - val[idx][1].header.stamp) > max_age:
                    idx_to_delete.append(idx)

            val = [x for i, x in enumerate(val) if i not in idx_to_delete]

            if len(val) == 0:
                frames_to_delete.append(frame_id)

            self.meas[frame_id] = val

        for frame_id in frames_to_delete:
            del self.meas[frame_id]

    @property
    def inst(self):

        inst = ObjInstance()
        inst.object_id = self.object_id
        inst.object_type = self.object_type

        w = []

        px = []
        py = []
        pz = []

        r = []
        p = []
        y = []

        for frame_id, val in self.meas.iteritems():

            if len(val) < 2:
                continue

            for idx in range(0, len(val)):

                v = val[idx]

                # distance normalized to 0, 1
                d = (v[0] - self.min_dist) / (self.max_dist - self.min_dist)

                # weight based on distance from object to sensor
                w_dist = (1.0 - d) ** 2  # (0, 1)

                # newer detections are more interesting
                w_age = (float(idx) / (len(val) - 1)) / 2 + 0.5  # (0.5, 1)

                w.append(w_dist * w_age)

                px.append(v[1].pose.position.x)
                py.append(v[1].pose.position.y)
                pz.append(v[1].pose.position.z)

                rpy = transformations.euler_from_quaternion(q2a(v[1].pose.orientation))

                r.append([cos(rpy[0]), sin(rpy[0])])
                p.append([cos(rpy[1]), sin(rpy[1])])
                y.append([cos(rpy[2]), sin(rpy[2])])

        if len(w) < self.min_meas_cnt:
            return None

        inst.pose.position.x = np.average(px, weights=w)
        inst.pose.position.y = np.average(py, weights=w)
        inst.pose.position.z = np.average(pz, weights=w)

        # TODO read table size from param
        inst.on_table = 0 < inst.pose.position.x < 1.5 and 0 < inst.pose.position.x < 0.7

        ar = np.average(r, axis=0, weights=w)
        ap = np.average(p, axis=0, weights=w)
        ay = np.average(y, axis=0, weights=w)

        fr = atan2(ar[1], ar[0])
        fp = atan2(ap[1], ap[0])
        fy = atan2(ay[1], ay[0])

        cur_rpy = [fr, fp, fy]

        a2q(inst.pose.orientation, transformations.quaternion_from_euler(*cur_rpy))

        # TODO "fix" roll and pitch so they are only 0, 90, 180 or 270 degrees
        # yaw (in table coordinates) may stay as it is

        for (key, value) in self.flags.iteritems():
            kv = KeyValue()
            kv.key = key
            kv.value = value
            inst.flags.append(kv)

        return inst

    def transform(self, ps):

        self.tfl.waitForTransform(
            self.target_frame, ps.header.frame_id, ps.header.stamp, rospy.Duration(0.5))

        return self.tfl.transformPose(self.target_frame, ps)


# "tracking" of static objects
class ArtSimpleTracker:
    def __init__(self, target_frame="marker"):

        self.target_frame = target_frame
        self.tfl = tf.TransformListener()
        self.lock = threading.Lock()
        self.detection_enabled = True
        self.sub = rospy.Subscriber(
            "/art/object_detector/object", InstancesArray, self.cb, queue_size=1)
        self.pub = rospy.Publisher(
            "/art/object_detector/object_filtered", InstancesArray, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        self.meas_max_age = rospy.Duration(5.0)
        self.prune_timer = rospy.Timer(rospy.Duration(1.0), self.prune_timer_cb)
        self.objects = {}
        self.br = tf.TransformBroadcaster()

        self.srv_set_flag = rospy.Service('/art/object_detector/flag/set', ObjectFlagSet, self.srv_set_flag_cb)
        self.srv_clear_flag = rospy.Service('/art/object_detector/flag/clear', ObjectFlagClear, self.srv_clear_flag_cb)
        self.srv_clear_all_flags = rospy.Service('/art/object_detector/flag/clear_all', Empty, self.srv_clear_all_flags_cb)

        self.use_forearm_cams = False
        self.forearm_cams = ("/l_forearm_cam_optical_frame", "/r_forearm_cam_optical_frame")
        self.srv_enable_forearm = rospy.Service('/art/object_detector/forearm/enable', Empty, self.srv_enable_forearm_cb)
        self.srv_disable_forearm = rospy.Service('/art/object_detector/forearm/disable', Empty, self.srv_disable_forearm_cb)
        self.srv_enable_detection = rospy.Service('/art/object_detector/all/enable', Empty,
                                                  self.srv_enable_detection_cb)
        self.srv_disable_detection = rospy.Service('/art/object_detector/all/disable', Empty,
                                                   self.srv_disable_detection_cb)

    def srv_enable_forearm_cb(self, req):

        rospy.loginfo("Enabling forearm cameras.")
        self.use_forearm_cams = True

        return EmptyResponse()

    def srv_disable_forearm_cb(self, req):

        rospy.loginfo("Disabling forearm cameras.")
        self.use_forearm_cams = False

        with self.lock:
            for object_id, obj in self.objects.iteritems():

                for cf in self.forearm_cams:

                    try:
                        del obj.meas[cf]
                    except KeyError:
                        pass

        return EmptyResponse()

    def srv_enable_detection_cb(self, req):

        rospy.loginfo("Enabling object detection.")
        self.detection_enabled = True

        return EmptyResponse()

    def srv_disable_detection_cb(self, req):

        rospy.loginfo("Disabling object detection.")
        self.detection_enabled = False

        with self.lock:
            for object_id, obj in self.objects.iteritems():

                for cf in self.forearm_cams:

                    try:
                        del obj.meas[cf]
                    except KeyError:
                        pass

        return EmptyResponse()

    def srv_clear_all_flags_cb(self, req):

        with self.lock:

            for k, v in self.objects.iteritems():

                v.flags = {}

        return EmptyResponse()

    def srv_clear_flag_cb(self, req):

        with self.lock:

            resp = ObjectFlagClearResponse()

            if req.object_id not in self.objects:

                resp.success = False
                resp.error = "Unknown object"
                return resp

            if req.key not in self.objects[req.object_id].flags:

                resp.success = False
                resp.error = "Unknown key"
                return resp

            del self.objects[req.object_id].flags[req.key]
            resp.success = True
            return resp

    def srv_set_flag_cb(self, req):

        with self.lock:

            # TODO should flag be remembered even if object is lost and then detected again?
            resp = ObjectFlagSetResponse()

            if req.object_id not in self.objects:

                resp.success = False
                resp.error = "Unknown object"
                return resp

            self.objects[req.object_id].flags[req.flag.key] = req.flag.value
            resp.success = True
            return resp

    def prune_timer_cb(self, event):

        with self.lock:

            now = rospy.Time.now()

            for k, v in self.objects.iteritems():

                v.prune_meas(now, self.meas_max_age)

    def timer_cb(self, event):

        with self.lock:

            ia = InstancesArray()
            ia.header.frame_id = self.target_frame
            ia.header.stamp = rospy.Time.now()

            objects_to_delete = []

            for k, v in self.objects.iteritems():

                inst = v.inst

                if inst is None:  # new object might not have enough measurements yet

                    # TODO fix it: this would keep objects which were detected only few times
                    if not v.new and not v.lost:  # object is no longer detected

                        v.lost = True
                        objects_to_delete.append(k)
                        ia.lost_objects.append(k)
                        continue

                    continue

                if v.new:
                    v.new = False
                    ia.new_objects.append(k)

                ia.instances.append(inst)

                self.br.sendTransform((inst.pose.position.x, inst.pose.position.y, inst.pose.position.z),
                                      q2a(inst.pose.orientation), ia.header.stamp, "object_id_" + inst.object_id, self.target_frame)

            # commented out in order to keep object flags even if object is lost for some time
            # for obj_id in objects_to_delete:
            #    del self.objects[obj_id]

            self.pub.publish(ia)

    def cb(self, msg):

        if not self.detection_enabled:
            return
        if not self.use_forearm_cams and msg.header.frame_id in self.forearm_cams:
            return

        if msg.header.frame_id == self.target_frame:
            rospy.logwarn_throttle(1.0, "Some detections are already in target frame!")
            return

        with self.lock:

            for inst in msg.instances:

                if inst.object_id in self.objects:

                    rospy.logdebug("Updating object: " + inst.object_id)

                else:

                    rospy.loginfo("Adding new object: " + inst.object_id)
                    self.objects[inst.object_id] = TrackedObject(self.target_frame, self.tfl, inst.object_id,
                                                                 inst.object_type)

                ps = PoseStamped()
                ps.header = msg.header
                ps.pose = inst.pose
                self.objects[inst.object_id].add_meas(ps)


if __name__ == '__main__':
    try:
        rospy.init_node('art_simple_tracker')
        ArtSimpleTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
