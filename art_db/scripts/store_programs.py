#!/usr/bin/env python

import rospy
from rospy_message_converter import json_message_converter
from art_msgs.msg import ProgramItem, ProgramHeader, ProgramBlock, Program
from art_msgs.srv import getProgram, getProgramRequest, getProgramResponse, getProgramHeaders, \
    getProgramHeadersRequest, getProgramHeadersResponse
from art_utils import ArtApiHelper
import rosbag
import json


class ProgramStore(object):

    def __init__(self):
        self.art = ArtApiHelper()

    def store_programs(self):
        program_headers = self.art.get_program_headers()
        programs = []
        file = open("JSONprograms_" + str(rospy.Time.now().to_nsec()), "w")
        bag = rosbag.Bag('BAGprograms_' + str(rospy.Time.now().to_nsec()) + '.bag', 'w')
        for header in program_headers:
            program = self.art.load_program(header.id)
            programs.append(program)
            to_print = json.loads(json_message_converter.convert_ros_message_to_json(program))
            file.write(">>>>>>>>>>>>>>>>>>>>>>>>>>    program id " + str(header.id) +
                       "   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n")
            file.write(json.dumps(to_print, indent=2, sort_keys=False))
            file.write("\n\n\n")
            bag.write("program", program)
        file.close()
        bag.close()


if __name__ == '__main__':
    try:
        rospy.init_node('store_programs', log_level=rospy.INFO)
        ps = ProgramStore()
        ps.store_programs()
    except KeyboardInterrupt:
        print("Shutting down")
