import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)


class Scene:
    """
    Base class for all Scene objects
    Contains the base functionalities and the functions that all derived classes need to implement
    """

    def __init__(self):
        self.is_interactive = False  # TODO: remove, it is deprecated
        self.build_graph = False  # Indicates if a graph for shortest path has been built
        self.floor_body_ids = []  # List of ids of the floor_heights

    def load(self):
        """
        Function to load all elements into physics engine and renderer
        The elements to load may include: floor, building, objects
        :return: A list of elements composing the scene, including the ground, the building and objects
        """
        raise NotImplementedError()

    def get_random_floor(self):
        """
        Sample a random floor among all existing floor_heights in the scene
        While Gibson v1 scenes can have several floor_heights, the EmptyScene, StadiumScene and scenes from iGibson
        have only a single floor
        :return: An integer between 0 and NumberOfFloors-1
        """
        return 0

    def get_random_point(self, floor=None, random_height=False):
        """
        Sample a random valid location in the given floor
        :param floor: integer indicating the floor, or None if randomly sampled
        :param random_height: if the height should be randomly sampled or not
        :return: A tuple of random floor and random valid point (3D) in that floor
        """
        raise NotImplementedError()

    def get_shortest_path(self, floor, source_world, target_world, entire_path=False):
        """
        Query the shortest path between two points in the given floor
        :param floor: Floor to compute shortest path in
        :param source_world: Initial location in world reference frame
        :param target_world: Target location in world reference frame
        :param entire_path: Flag indicating if the function should return the entire shortest path or not
        :return: Tuple of path (if indicated) as a list of points, and geodesic distance (lenght of the path)
        """
        raise NotImplementedError()

    def get_floor_height(self, floor=0):
        """
        Get the height of the given floor
        :param floor: Integer identifying the floor
        :return: Height of the given floor
        """
        del floor
        return 0.0

    def reset_floor(self, floor=0, additional_elevation=0.02, height=None):
        """
        Resets the ground plane to a new floor
        :param floor: Integer identifying the floor to move the ground plane to
        :param additional_elevation: Additional elevation with respect to the height of the floor
        :param height: Alternative parameter to control directly the height of the ground plane
        :return: None
        """
        return