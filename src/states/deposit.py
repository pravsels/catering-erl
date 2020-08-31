#!/usr/bin/env python
import rospy
from smach import State
from geometry_msgs.msg import Pose, Point
from utilities import Util

class Deposit(State):
    def __init__(self, tiago):
        State.__init__(self, outcomes=['fetch_more', 'deposit_done'])
        self.tiago = tiago

    def go_to_user_location(self):
        # go to approach point, orientation will be off
        self.tiago.goto_location(Pose(self.tiago.approach_user_point, Util.quaternion_at_point(self.tiago.approach_user_point)))
        # orient to face table user point
        self.tiago.goto_location(Util.turn_towards_point(Point(self.tiago.user_point.x, self.tiago.user_point.y, 0.0)))


    def execute(self, userdata):
        search_n_fetch_queries = rospy.get_param('/search_n_fetch_queries')
        search_n_fetch_requests = self.tiago.search_n_fetch_requests
        print('in deposit state')
        print(search_n_fetch_requests)

        search_reqs = filter(lambda q: q['intent'] == 'search', search_n_fetch_requests)
        fetch_reqs = filter(lambda q: q['intent'] == 'fetch', search_n_fetch_requests)

        self.go_to_user_location()

        if len(search_reqs) or len(search_n_fetch_queries):
            # report on found objects
            for search_req in search_reqs:
                object = search_req['object']
                furniture = search_req['furniture']
                report_sentence = 'I found ' + object.name + ' on ' + furniture['name']
                self.tiago.talk(report_sentence)

            # report on failed to find objects
            for search_q in search_n_fetch_queries:
                entity = search_q['entities'][0]
                report_sentence = 'I was not able to find ' + entity
                self.tiago.talk(report_sentence)


        if len(fetch_reqs):
            # if attempted pick fails, then report its position and remove it from fetch list
            for fetch_req in fetch_reqs:
                if fetch_req['status'] == 'pick_failed':
                    object = fetch_req['object']
                    furniture = fetch_req['furniture']
                    report_sentence = 'I was not able to fetch ' + object.name + ', but I found it on ' + furniture['name']
                    self.tiago.talk(report_sentence)
                    fetch_reqs.remove(fetch_req)
                elif fetch_req['status'] == 'pick_success':
                    object = fetch_req['object']
                    report_sentence = 'Please take this ' + object.name + ' from me.'
                    self.tiago.talk(report_sentence)
                    self.tiago.play('open_gripper')
                    fetch_reqs.remove(fetch_req)

            # all search reqs would be satisfied and failed fetch reqs would be removed
            # so store the remaiming fetch reqs
            if len(fetch_reqs):
                self.tiago.search_n_fetch_requests = fetch_reqs
                return 'fetch_more'

        return 'deposit_done'
