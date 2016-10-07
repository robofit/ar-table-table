#!/usr/bin/env python

from transitions.extensions import HierarchicalGraphMachine as Machine
import logging
from transitions import logger

class FSM(Machine):

    def __init__(self):

        super(FSM, self).__init__(show_conditions=True,  auto_transitions=False,  title="Projected GUI state machine")

        states = ['initial', 'calibrate', 'waiting_for_user',  'waiting_for_user_calibration',  'program_selection', 'learning', 'running']

        self.add_states(states)

        logger.setLevel(logging.DEBUG)

        # trigger, source, destination
        self.add_transition('tr_start', 'initial', 'calibrate',  after='cb_start_calibration')
        self.add_transition('tr_calibrated',  'calibrate', 'waiting_for_user',  after='cb_waiting_for_user')
        self.add_transition('tr_user_present', 'waiting_for_user', 'waiting_for_user_calibration',  after="cb_waiting_for_user_calibration")
        self.add_transition('tr_user_calibrated', 'waiting_for_user_calibration', 'program_selection',  after='cb_program_selection')
        self.add_transition('tr_program_selected', 'program_selection', 'learning',  conditions='is_template',  after="cb_learning")
        self.add_transition('tr_program_selected', 'program_selection', 'running',  unless='is_template')


    def is_template(self):

        pass

    def cb_learning(self):

        pass

    def cb_start_calibration(self):

        pass

    def cb_waiting_for_user(self):

        pass

    def cb_program_selection(self):

        pass

    def cb_waiting_for_user_calibration(self):

        pass

    def draw_graph(self):

        self.graph.draw('state_diagram.png', prog='dot')

def main():

    fsm = FSM()
    fsm.draw_graph()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
