#!/usr/bin/env python
import rospy
from utilities import Tiago, OccupancyGridMap, Search
from smach import State, StateMachine
from states import Start, Explore, Manipulate, Deposit


if __name__ == '__main__':
    rospy.init_node('butler')
    tiago = Tiago()
    ogm = OccupancyGridMap()
    search = Search(dataset='costa')

    tiago.play('home')

    sm = StateMachine(outcomes=['success', 'failure'])  # the end states of the machine

    with sm:
        StateMachine.add('start', Start(tiago, ogm), transitions={'start_done':'explore', 'start_abort': 'failure'})
        StateMachine.add('explore', Explore(tiago, ogm, search), transitions={'manipulate_object': 'manipulate', 'report_back': 'deposit'}, remapping={'explore_out': 'search_n_fetch_requests'})
        StateMachine.add('manipulate', Manipulate(tiago, ogm, search), transitions={'manipulate_done':'deposit'}, remapping={'manipulate_in': 'search_n_fetch_requests', 'manipulate_out': 'search_n_fetch_requests'})
        StateMachine.add('deposit', Deposit(tiago), transitions={'fetch_more':'manipulate', 'deposit_done': 'success'}, remapping={'deposit_in': 'search_n_fetch_requests', 'deposit_out': 'search_n_fetch_requests'})
        sm.execute()


    rospy.spin()
