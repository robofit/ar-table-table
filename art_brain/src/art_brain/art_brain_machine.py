# from transitions import Machine
# from transitions.extensions import GraphMachine as Machine
from transitions.extensions import LockedMachine as Machine

from transitions import State


class ArtBrainMachine(object):
    states = [State(name='pre_init', on_enter=[], on_exit=[]),
              State(name='init', on_enter=['state_init_ros'], on_exit=[]),
              State(name='shutdown', on_enter=['state_shutdown'], on_exit=[]),
              State(name='waiting_for_action', on_enter=[
                    'state_waiting_for_action'], on_exit=[]),
              State(name='program_init', on_enter=[
                    'state_program_init'], on_exit=[]),
              State(name='program_run', on_enter=[
                    'state_program_run'], on_exit=[]),
              State(name='program_paused', on_enter=[
                    'state_program_paused'], on_exit=[]),

              # basic instructions
              State(name='get_ready', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_get_ready'], on_exit=['check_robot_out']),

              # synchronization with the user
              State(name='wait_for_user', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_wait_for_user'], on_exit=['check_robot_out']),
              State(name='wait_until_user_finishes', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_wait_until_user_finishes'], on_exit=['check_robot_out']),

              # manipulation - pick
              State(name='pick_from_polygon', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_pick_from_polygon'], on_exit=['check_robot_out']),
              State(name='pick_from_feeder', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_pick_from_feeder'], on_exit=['check_robot_out']),
              State(name='pick_object_id', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_pick_object_id'], on_exit=['check_robot_out']),

              # manipulation - place
              State(name='place_to_pose', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_place_to_pose'], on_exit=['check_robot_out']),
              State(name='place_to_grid', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_place_to_grid'], on_exit=['check_robot_out']),

              # manipulation
              State(name='path_through_points', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_path_through_points'], on_exit=['check_robot_out']),
              State(name='welding_points', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_welding_points'], on_exit=['check_robot_out']),
              State(name='welding_seam', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_welding_seam'], on_exit=['check_robot_out']),
              State(name='drill_points', on_enter=[
                    'state_update_program_item', 'check_robot_in', 'state_drill_points'], on_exit=['check_robot_out']),

              State(name='program_error', on_enter=[
                    'state_program_error'], on_exit=[]),
              State(name='program_finished', on_enter=[
                    'state_program_finished'], on_exit=[]),
              State(name='program_load_instruction', on_enter=[
                    'state_program_load_instruction'], on_exit=[]),

              # learning
              State(name='learning_init', on_enter=[
                    'learning_load_block_id', 'state_learning_init'], on_exit=[]),
              State(name='learning_run', on_enter=[
                    'learning_load_block_id', 'state_learning_run'], on_exit=[]),

              # learning picking
              State(name='learning_pick_from_polygon', on_enter=[
                    'learning_load_block_id', 'state_learning_pick_from_polygon'], on_exit=[]),
              State(name='learning_pick_from_feeder', on_enter=[
                    'check_robot_in', 'learning_load_block_id', 'state_learning_pick_from_feeder'],
                    on_exit=['check_robot_out', 'state_learning_pick_from_feeder_exit']),
              State(name='learning_pick_object_id', on_enter=[
                    'learning_load_block_id', 'state_learning_pick_object_id'], on_exit=[]),

              State(name='learning_pick_from_polygon_run', on_enter=[
                    'check_robot_in', 'learning_load_block_id', 'state_learning_pick_from_polygon_run'],
                    on_exit=['check_robot_out']),
              State(name='learning_pick_from_feeder_run', on_enter=[
                  'check_robot_in', 'learning_load_block_id', 'state_learning_pick_from_feeder_run'],
        on_exit=['check_robot_out']),
        State(name='learning_pick_object_id_run', on_enter=[
            'learning_load_block_id', 'state_learning_pick_object_id_run'], on_exit=[]),

        # learning placing
        State(name='learning_place_to_pose', on_enter=[
            'learning_load_block_id', 'state_learning_place_to_pose'], on_exit=[]),
        State(name='learning_place_to_pose_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_place_to_pose_run'],
        on_exit=['check_robot_out']),
        State(name='learning_place_to_grid', on_enter=[
            'learning_load_block_id', 'state_learning_place_to_grid'], on_exit=[]),

        # learning drilling and welding
        State(name='learning_welding_point', on_enter=[
            'learning_load_block_id', 'state_learning_welding_point'], on_exit=[]),
        State(name='learning_welding_point_run', on_enter=[
            'learning_load_block_id', 'state_learning_welding_point_run'], on_exit=[]),

        State(name='learning_welding_seam', on_enter=[
            'learning_load_block_id', 'state_learning_welding_seam'], on_exit=[]),
        State(name='learning_welding_seam_run', on_enter=[
            'learning_load_block_id', 'state_learning_welding_seam_run'], on_exit=[]),

        State(name='learning_drill_points', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_drill_points'], on_exit=[
            'check_robot_out', 'state_learning_drill_points_exit']),
        State(name='learning_drill_points_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_drill_points_run'],
        on_exit=['check_robot_out']),

        # learning others

        State(name='learning_wait', on_enter=[
            'learning_load_block_id', 'state_learning_wait'], on_exit=[]),
        State(name='learning_step_done', on_enter=[
            'learning_load_block_id', 'state_learning_step_done'], on_exit=[]),
        State(name='learning_step_error', on_enter=[
            'learning_load_block_id', 'state_learning_step_error'], on_exit=[]),
        State(name='learning_done', on_enter=[
            'learning_load_block_id', 'state_learning_done'], on_exit=[])]

    def __init__(self):
        self.name = 'brain'
        self.machine = Machine(model=self, states=ArtBrainMachine.states, initial='pre_init',
                               auto_transitions=False, send_event=True, queued=True)

        # *** transitions ***

        self.machine.add_transition('init', 'pre_init', 'init')
        self.machine.add_transition(
            'init_done', 'init', 'waiting_for_action', conditions='is_everything_calibrated')
        self.machine.add_transition(
            'program_start', 'waiting_for_action', 'program_init')
        self.machine.add_transition(
            'learning_start', 'waiting_for_action', 'learning_init')

        # program
        self.machine.add_transition(
            'program_init_done', 'program_init', 'program_run')
        self.machine.add_transition('error', 'program_init', 'program_error')
        self.machine.add_transition('error', 'program_run', 'program_error')
        self.machine.add_transition(
            'program_error_handled', 'program_error', 'program_load_instruction')
        self.machine.add_transition(
            'program_error_shutdown', 'program_error', 'shutdown')
        self.machine.add_transition(
            'program_error_fatal', 'program_error', 'program_finished')
        self.machine.add_transition(
            'try_again', 'program_error', 'program_run')
        self.machine.add_transition(
            'skip', 'program_error', 'program_load_instruction')
        self.machine.add_transition(
            'fail', 'program_error', 'program_load_instruction')
        self.machine.add_transition(
            'done', 'program_load_instruction', 'program_run')
        self.machine.add_transition(
            'error', 'program_load_instruction', 'program_error')
        self.machine.add_transition(
            'finished', 'program_load_instruction', 'program_finished')
        self.machine.add_transition(
            'done', 'program_finished', 'waiting_for_action')
        self.machine.add_transition(
            'finished', 'program_run', 'program_finished')
        self.machine.add_transition(
            'pause', 'program_load_instruction', 'program_paused')
        self.machine.add_transition(
            'resume', 'program_paused', 'program_run')

        # get ready instruction
        self.machine.add_transition('get_ready', 'program_run', 'get_ready')
        self.machine.add_transition(
            'done', 'get_ready', 'program_load_instruction')
        self.machine.add_transition('error', 'get_ready', 'program_error')

        # pick_from_polygon instruction
        self.machine.add_transition(
            'pick_from_polygon', 'program_run', 'pick_from_polygon')
        self.machine.add_transition(
            'done', 'pick_from_polygon', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'pick_from_polygon', 'program_error')

        # pick_from_feeder instruction
        self.machine.add_transition(
            'pick_from_feeder', 'program_run', 'pick_from_feeder')
        self.machine.add_transition(
            'done', 'pick_from_feeder', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'pick_from_feeder', 'program_error')

        # pick_object_id instruction
        self.machine.add_transition(
            'pick_object_id', 'program_run', 'pick_object_id')
        self.machine.add_transition(
            'done', 'pick_object_id', 'program_load_instruction')
        self.machine.add_transition('error', 'pick_object_id', 'program_error')

        # place_to_pose instruction
        self.machine.add_transition(
            'place_to_pose', 'program_run', 'place_to_pose')
        self.machine.add_transition(
            'done', 'place_to_pose', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'place_to_pose', 'program_error')

        # place_to_grid instruction
        self.machine.add_transition(
            'place_to_grid', 'program_run', 'place_to_grid')
        self.machine.add_transition(
            'done', 'place_to_grid', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'place_to_grid', 'program_error')

        # path through poses instruction
        self.machine.add_transition(
            'path_through_points', 'program_run', 'path_through_points')
        self.machine.add_transition(
            'done', 'path_through_points', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'path_through_points', 'program_error')

        # path through poses instruction
        self.machine.add_transition(
            'welding_points', 'program_run', 'welding_points')
        self.machine.add_transition(
            'done', 'welding_points', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'welding_points', 'program_error')

        # path through poses instruction
        self.machine.add_transition(
            'welding_seam', 'program_run', 'welding_seam')
        self.machine.add_transition(
            'done', 'welding_seam', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'welding_seam', 'program_error')

        # path through poses instruction
        self.machine.add_transition(
            'drill_points', 'program_run', 'drill_points')
        self.machine.add_transition(
            'done', 'drill_points', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'drill_points', 'program_error')

        # wait instruction
        self.machine.add_transition(
            'wait_for_user', 'program_run', 'wait_for_user')
        self.machine.add_transition(
            'done', 'wait_for_user', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'wait_for_user', 'program_error')

        # wait instruction
        self.machine.add_transition(
            'wait_until_user_finishes', 'program_run', 'wait_until_user_finishes')
        self.machine.add_transition(
            'done', 'wait_until_user_finishes', 'program_load_instruction')
        self.machine.add_transition(
            'error', 'wait_until_user_finishes', 'program_error')

        #
        # learning
        #

        self.machine.add_transition(
            'init_done', 'learning_init', 'learning_run')
        self.machine.add_transition(
            'done', 'learning_step_done', 'learning_run')
        self.machine.add_transition(
            'error_handled', 'learning_step_error', 'learning_run')
        self.machine.add_transition(
            'error_fatal', 'learning_step_error', 'waiting_for_action')
        self.machine.add_transition(
            'done', 'learning_done', 'waiting_for_action')
        self.machine.add_transition(
            'learning_done', 'learning_run', 'learning_done')
        # learning pick_from_polygon
        self.machine.add_transition(
            'pick_from_polygon', 'learning_run', 'learning_pick_from_polygon')
        self.machine.add_transition(
            'done', 'learning_pick_from_polygon', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_pick_from_polygon', 'learning_step_error')
        self.machine.add_transition(
            'pick_from_polygon_run', 'learning_run', 'learning_pick_from_polygon_run')
        self.machine.add_transition(
            'done', 'learning_pick_from_polygon_run', 'learning_run')
        self.machine.add_transition(
            'error', 'learning_pick_from_polygon_run', 'learning_step_error')

        # learning pick_from_feeder
        self.machine.add_transition(
            'pick_from_feeder', 'learning_run', 'learning_pick_from_feeder')
        self.machine.add_transition(
            'done', 'learning_pick_from_feeder', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_pick_from_feeder', 'learning_step_error')
        self.machine.add_transition(
            'pick_from_feeder_run', 'learning_run', 'learning_pick_from_feeder_run')
        self.machine.add_transition(
            'done', 'learning_pick_from_feeder_run', 'learning_run')
        self.machine.add_transition(
            'error', 'learning_pick_from_feeder_run', 'learning_step_error')

        # learning pick_object_id
        self.machine.add_transition(
            'pick_object_id', 'learning_run', 'learning_pick_object_id')
        self.machine.add_transition(
            'done', 'learning_pick_object_id', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_pick_object_id', 'learning_step_error')
        self.machine.add_transition(
            'pick_object_id_run', 'learning_run', 'learning_pick_object_id_run')
        self.machine.add_transition(
            'done', 'learning_pick_object_id_run', 'learning_run')
        self.machine.add_transition(
            'error', 'learning_pick_object_id', 'learning_step_error')

        # learning place_to_pose
        self.machine.add_transition(
            'place_to_pose', 'learning_run', 'learning_place_to_pose')
        self.machine.add_transition(
            'done', 'learning_place_to_pose', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_place_to_pose', 'learning_step_error')
        self.machine.add_transition(
            'place_to_pose_run', 'learning_run', 'learning_place_to_pose_run')
        self.machine.add_transition(
            'done', 'learning_place_to_pose_run', 'learning_run')
        self.machine.add_transition(
            'error', 'learning_place_to_pose_run', 'learning_step_error')

        # learning welding_point
        self.machine.add_transition(
            'welding_point', 'learning_run', 'learning_welding_point')
        self.machine.add_transition(
            'done', 'learning_welding_point', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_welding_point', 'learning_step_error')
        self.machine.add_transition(
            'welding_point_run', 'learning_run', 'learning_welding_point_run')
        self.machine.add_transition(
            'done', 'learning_welding_point_run', 'learning_run')
        self.machine.add_transition(
            'error', 'learning_welding_point_run', 'learning_step_error')

        # learning welding_seam
        self.machine.add_transition(
            'welding_seam', 'learning_run', 'learning_welding_seam')
        self.machine.add_transition(
            'done', 'learning_welding_seam', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_welding_seam', 'learning_step_error')
        self.machine.add_transition(
            'welding_seam_run', 'learning_run', 'learning_welding_seam_run')
        self.machine.add_transition(
            'done', 'learning_welding_seam_run', 'learning_run')
        self.machine.add_transition(
            'error', 'learning_welding_seam_run', 'learning_step_error')

        # learning drill_points
        self.machine.add_transition(
            'drill_points', 'learning_run', 'learning_drill_points')
        self.machine.add_transition(
            'done', 'learning_drill_points', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_drill_points', 'learning_step_error')
        self.machine.add_transition(
            'drill_points_run', 'learning_run', 'learning_drill_points_run')
        self.machine.add_transition(
            'done', 'learning_drill_points_run', 'learning_run')
        self.machine.add_transition(
            'error', 'learning_drill_points_run', 'learning_step_error')

        # learning place_to_grid
        self.machine.add_transition(
            'place_to_grid', 'learning_run', 'learning_place_to_grid')
        self.machine.add_transition(
            'done', 'learning_place_to_grid', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_place_to_grid', 'learning_step_error')

        # learning wait
        self.machine.add_transition('wait', 'learning_run', 'learning_wait')
        self.machine.add_transition(
            'done', 'learning_wait', 'learning_step_done')
        self.machine.add_transition(
            'error', 'learning_wait', 'learning_step_error')

        # learning pick_from_feeder
        # self.machine.add_transition('pick_from_feeder', 'learning_pick', 'learning_pick_from_feeder')
        # self.machine.add_transition('pick__from_feeder', 'learning_pick_place', 'learning_pick_from_feeder')
        # self.machine.add_transition('done_pick', 'learning_pick_from_feeder', 'learning_pick')
        # self.machine.add_transition('done_pick_place', 'learning_pick_from_feeder', 'learning_pick_place')
        # self.machine.add_transition('error', 'learning_pick_from_feeder', 'learning_step_error')

        # self.graph.draw('my_state_diagram.png', prog='dot')
        # return
