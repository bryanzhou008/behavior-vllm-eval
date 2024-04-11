class Transition_model:
    def __init__(self):
        pass
    

    def get_simulator_state(self,simulator):
        object_dump = {}
        for name, obj in simulator.scene.objects_by_name.items():
            object_dump[name] = obj.dump_state()

        # Dump the robot state.
        robot_dump = []
        for robot in simulator.robots:
            robot_dump.append(robot.dump_state())

        return {"objects": object_dump, "robots": robot_dump}
    
    def load_simulator_state(self,simulator,dump):
        object_dump = dump["objects"]
        for name, obj in simulator.scene.objects_by_name.items():
            obj.load_state(object_dump[name])

        # Restore the robot state.
        robot_dumps = dump["robots"]
        for robot, robot_dump in zip(simulator.robots, robot_dumps):
            robot.load_state(robot_dump)

    def apply_action(self,action,simulator):
        # apply action to simulator and return the next state
        pass

    def step():
        pass
    




