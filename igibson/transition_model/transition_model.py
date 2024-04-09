class Transition_model:
    def __init__(self):
        pass
    

    def get_simulator_state(self):
        # convert simulator state to a dictionary
        pass

    def apply_action(self,action):
        # apply action to simulator and return the next state
        pass

    

class MotionPlanner:
    def __init__(self,activity, activity_id):
        pass

    def get_initial_state(self):
        # return initial state of of a task scene
        pass

    def get_goal_condition(self):
        # return goal condition of a task
        pass
    
    @staticmethod
    def get_relavant_objects(demo):
        # return relevant objects in the scene
        # three methods to get relavnt objects
        # objects included in the goal condition/ diff between initial and goal state
        # objects touched by the agent
        # objects gazed at in a significant amount of time
        pass


