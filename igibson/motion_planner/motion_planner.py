import bddl
from bddl.activity import Conditions, get_initial_conditions, get_goal_conditions, get_object_scope
# from bddl.activity import *
import json


class MotionPlanner:
    def __init__(self, 
                 behavior_activity, 
                 activity_definition, 
                 simulator_name):
        self.behavior_activity = behavior_activity 
        self.activity_definition = activity_definition
        self.simulator_name = simulator_name
        
        self.conditions = Conditions(behavior_activity, activity_definition, simulator_name)
        self.conditions.object_scope = get_object_scope(self.conditions)


    def get_initial_state(self):
        """
        Returns a JSON string containing human-readable and machine-readable initial states of the activity.
        """
        
        initial_state_data = {
            "human_readable": self.conditions.parsed_initial_conditions,
            "machine_readable": get_initial_conditions(self.conditions)
        }

        return json.dumps(initial_state_data, indent=4)

        
        
    def get_goal_condition(self):
        """
        Returns a JSON string containing human-readable and machine-readable goal conditions of the activity.
        """
        goal_condition_data = {
            "human_readable": self.conditions.parsed_goal_conditions,
            "machine_readable": get_goal_conditions(self.conditions, self.conditions.object_scope)
        }

        return json.dumps(goal_condition_data, indent=4)
    
    
    @staticmethod
    def bddl_get_relavant_objects(self):
        """
        Returns a JSON string containing relevant objects from bddl definition, key is object type and value includes all relevant instances of that object type
        """
        relavant_objects = self.conditions.parsed_objects
        
        return json.dumps(relavant_objects, indent=4)
        
        
        
    
    
    
    
    @staticmethod
    def igibson_get_relavant_objects(self, demo):
        # return relevant objects in the scene
        # three methods to get relavnt objects
        # objects included in the goal condition/ diff between initial and goal state
        # objects touched by the agent
        # objects gazed at in a significant amount of time
        
        
        
        pass


