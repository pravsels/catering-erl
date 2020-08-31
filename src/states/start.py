#!/usr/bin/env python
import rospy
from smach import State
import uuid
import dialogflow_v2 as dialogflow
from utilities import Util
from geometry_msgs.msg import Pose, Point

class Start(State):
    def __init__(self, tiago, ogm):
        State.__init__(self, outcomes=['start_done', 'start_abort'])
        self.tiago = tiago
        self.ogm = ogm
        self.dialogflow_project_id = rospy.get_param('/dialogflow_project_id')
        self.session_id = str(uuid.uuid4())
        self.speech_mode = 'audio'


    def get_intent_from_text(self, project_id, session_id, text, language_code='en-US'):
        # get intents and entities from text
        session_client = dialogflow.SessionsClient()
        session = session_client.session_path(project_id, session_id)

        text_input = dialogflow.types.TextInput(text=text, language_code=language_code)
        query_input = dialogflow.types.QueryInput(text=text_input)

        response = session_client.detect_intent(session=session, query_input=query_input)

        return response


    def filter_out_empty_entities(self, entities):
        # filters out empty entity parameters from dialogflow result
        filtered_objects = []
        for key, value in entities.items():
            if not value:
                continue
            else:
                filtered_objects.append(str(value))

        return filtered_objects


    def get_result_from_dialogflow(self):
        speech = self.tiago.accept_speech(mode=self.speech_mode)
        rospy.loginfo(speech)

        text = speech['transcription']
        response = self.get_intent_from_text(self.dialogflow_project_id, self.session_id, text)
        entities = self.filter_out_empty_entities(response.query_result.parameters)

        search_n_fetch_query = {
            'intent': response.query_result.intent.display_name,
            'intent_confidence': response.query_result.intent_detection_confidence,
            'entities': entities
        }
        rospy.loginfo(search_n_fetch_query)

        return search_n_fetch_query


    def single_exchange(self):

        while True:
            snf_query = self.get_result_from_dialogflow()

            if snf_query['intent'] and len(snf_query['entities']):
                self.tiago.talk('You would like me to ' + snf_query['intent'] + ' ' + snf_query['entities'][0])
                self.tiago.talk('Is that correct ?')
                speech = self.tiago.accept_speech(mode=self.speech_mode)
                text = speech['transcription']
                if 'yes' in text.split():
                    return snf_query
                else:
                    self.tiago.talk('Please repeat your request.')
            else:
                self.tiago.talk('I did not get that, can you say it again please ?')


    def extract_tasks_with_dialogue(self):

        search_n_fetch_queries = []
        dialogue_exchange = True

        while dialogue_exchange:
            snf_query = self.single_exchange()
            search_n_fetch_queries.append(snf_query)
            self.tiago.talk('Anything else ?')
            speech = self.tiago.accept_speech(mode=self.speech_mode)
            text = speech['transcription']
            print(text)
            if 'yes' in text.split():
                self.tiago.talk('Go ahead.')
                continue
            else:
                dialogue_exchange = False


        return search_n_fetch_queries


    def execute(self, userdata):
        # go to a specified location where the user is at
        approach_user_point = self.ogm.approach_user(self.tiago.user_point)
        self.tiago.approach_user_point = approach_user_point

        # go to approach point, orientation will be off
        self.tiago.goto_location(Pose(approach_user_point, Util.quaternion_at_point(approach_user_point)))
        # orient to face table user point
        self.tiago.goto_location(Util.turn_towards_point(Point(self.tiago.user_point.x, self.tiago.user_point.y, 0.0)))

        self.tiago.talk('Hello, what would you like me to do ?')

        search_n_fetch_queries = self.extract_tasks_with_dialogue()

        if len(search_n_fetch_queries):
            print(search_n_fetch_queries)
            rospy.set_param('/search_n_fetch_queries', search_n_fetch_queries)
            self.tiago.talk('I will be right back, with your requests.')
            return 'start_done'
        else:
            self.tiago.talk('I did not get any requests from you. I will go back to sleep now.')
            return 'start_abort'
