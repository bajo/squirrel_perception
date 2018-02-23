#!/usr/bin/env python

import rospy
import smach
import uashh_smach
import smach_ros
import std_msgs.msg
import numpy as np


from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from squirrel_object_perception_msgs.msg import FindLumpsAction, FindLumpsGoal, LookForObjectsAction, \
    LookForObjectsGoal, LookForObjectsResult, SceneObject
from squirrel_manipulation_msgs.msg import ManipulationAction, ManipulationGoal

from smach import Sequence, StateMachine
from smach_ros import SimpleActionState, ServiceState
from actionlib_msgs.msg import GoalStatus

import uashh_smach.platform.move_base as move_base

NODE_NAME = 'squirrel_ikea_demo'
START_POSE = (1.0, 2.0, 1.5)
WAYPOINTS = [(1.0, 2.0, 1.5),
             (2.0, 2.0, 1.5),
             (2.0, 2.0, 0.0)
             ]

def check_for_faces(msg, ud):
    return msg.data

def check_for_start_command(msg, ud):
    return msg.data

def calculate_distance_between_poses(pose1, pose2):
    tl = tf.TransformListener()
    try:
        tl.waitForTransform(pose1.header.frame_id, pose2.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        pose = tl.transformPose(pose1.header.frame_id, pose2)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return None

    a = np.array((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    b = np.array((pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z))
    dist = np.linalg.norm(a-b)
    return dist


@smach.cb_interface(input_keys=['lumps'],
                    output_keys=['lump'],
                    outcomes=['succeeded', 'aborted'])
def my_cb(ud):
    if len(ud.lumps) == 0:
        return 'aborted'
    base_pose = PoseStamped()
    base_pose.header.frame_id = 'base_link'
    distances = []
    for i in ud.lumps:
        lump_pose = PoseStamped(i.header, i.pose)
        dist = calculate_distance_between_poses(base_pose, lump_pose)
        distances.append(dist)
    print(distances)
    a = np.array(distances)
    index = np.where(a == a.min())
    print(index)
    ud.lump = ud.lumps[index]
    return 'succeeded'

def get_closest_lump(ud):
    pass


def start_demo():
    seq = Sequence(outcomes=['aborted', 'preempted', 'succeeded'],
                   connector_outcome='succeeded')

    sm = StateMachine(outcomes=['aborted', 'preempted', 'succeeded'])

    with seq:
        # Move to the start pose
        Sequence.add('MOVE_TO_START',
                     move_base.get_move_base_state(frame='/map',
                                                   x=START_POSE[0],
                                                   y=START_POSE[1],
                                                   yaw=START_POSE[2])
                     )
        Sequence.add('WAIT_FOR_FACES',
                     uashh_smach.util.WaitForMsgState('/ikea/face_deteced',
                                                      std_msgs.msg.Bool,
                                                      msg_cb = check_for_faces,
                                                      timeout = 10
                                                      ),
                     transitions={'aborted': 'WAIT_FOR_FACES'})
        Sequence.add('WAIT_FOR_START_COMMAND',
                     uashh_smach.util.WaitForMsgState('/ikea/start_command',
                                                      std_msgs.msg.Bool,
                                                      msg_cb=check_for_start_command,
                                                      timeout=10
                                                      ),
                     transitions={'aborted': 'WAIT_FOR_START_COMMAND'})



        def lumps_goal_cb(userdata, goal):
            ret = FindLumpsGoal()
            for i in WAYPOINTS:
                pose = Pose()
                pose.position.x = i[0]
                pose.position.y = i[1]
                pose.position.z = 0.0
                quat = Quaternion()
                quat = quaternion_from_euler(0.0, 0.0, i[2])
                pose.orientation = Quaternion(*quat)
                ret.waypointPoses.poses.append(pose)
            ret.waypointPoses.header.frame_id = '/map'
            ret.return_after_first_lump = True
            print(ret)
            return ret

        def lump_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.lumps = result.lumps_found
                return 'succeeded'
        Sequence.add('GET_LIST_OF_LUMP_POSES',
                     SimpleActionState('squirrel_find_lumps_via_waypoints',
                                       FindLumpsAction,
                                       goal_cb=lumps_goal_cb,
                                       result_cb=lump_result_cb,
                                       output_keys=['lump_pose'])
                     )
        def prepare_haf_goal_cb(userdata, goal):
            goal = ManipulationGoal()
            goal.manipulation_type = 'prepare haf'
            #goal.pose = pose
            #goal.object_bounding_cylinder = cylinder
            return goal
        Sequence.add('PREPARE_HAF',
                     SimpleActionState('prepare_haf',
                                       ManipulationAction,
                                       goal_cb=prepare_haf_goal_cb)
                     )

        def recognize_object_goal_cb(userdata, goal):
            ret_goal = LookForObjectsGoal()
            ret_goal.look_for_object = LookForObjectsGoal.EXPLORE
            ret_goal.pose = userdata.lump_pose
            return ret_goal
        Sequence.add('RECOGNIZE_OBJECT',
                     SimpleActionState('squirrel_recognize_objects',
                                       LookForObjectsAction,
                                       goal_cb=recognize_object_goal_cb,
                                       input_keys=['lump_pose'],
                                       output_keys=['recognized_object']
                                       )
                     )

        def haf_grasp_goal_cb(userdata, goal):
            goal = ManipulationGoal()
            goal.manipulation_type = 'haf pick'
            rec = LookForObjectsResult()
            goal.pose = rec.objects_added[0].pose
            goal.object_bounding_cylinder = rec.objects_added[0].bounding_cylinder
            return goal
        Sequence.add('HAF_GRASPING',
                     SimpleActionState('prepare_haf',
                                       ManipulationAction,
                                       goal_cb=haf_grasp_goal_cb,
                                       input_keys=['recognized_object'])
                     )

        def place_object_goal_cb(userdata, goal):
            goal = ManipulationGoal()
            goal.manipulation_type = 'place'
            return goal
        Sequence.add('STORE_OBJECT',
                     SimpleActionState('prepare_haf',
                                       ManipulationAction,
                                       goal_cb=place_object_goal_cb)
                     )

        Sequence.add('RETURN_TO_START',
                     move_base.get_move_base_state(frame='/map',
                                                   x=START_POSE[0],
                                                   y=START_POSE[1],
                                                   yaw=START_POSE[2])
                     )


    with sm:
        StateMachine.add('SEQUENCE', seq)

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    rospy.loginfo("{} started".format(NODE_NAME))
    start_demo()
