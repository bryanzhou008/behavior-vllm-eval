import numpy as np
from pyquaternion import Quaternion
from igibson.objects.articulated_object import URDFObject
import igibson.object_states as object_states
from igibson.object_states.utils import clear_cached_states, sample_kinematics

def get_aabb_center(obj1:URDFObject):
    lo,hi=obj1.states[object_states.AABB].get_value()
    return (np.array(lo) + np.array(hi)) / 2.

def get_aabb(obj1:URDFObject):
    lo,hi=obj1.states[object_states.AABB].get_value()
    return abs(hi-lo)

def tar_pos_for_new_aabb_center(obj1:URDFObject,new_center:np.ndarray):
    cur_pos=obj1.get_position()
    cur_aabb_center=get_aabb_center(obj1)
    delta=new_center-cur_aabb_center
    return cur_pos+delta
    

class PositionGeometry:

    def __init__(self,robot,using_kinematics=False):
        self.robot=robot
        self.robot.bounding_box=[0.5, 0.5, 1]
        self.using_kinematics=using_kinematics

    # high failure rate
    def _set_inside_kinematics(self,obj1:URDFObject,obj2:URDFObject):
        return obj1.states[object_states.Inside].set_value(obj2,True)
    
    def _set_ontop_kinematics(self,obj1:URDFObject,obj2:URDFObject):
        return obj1.states[object_states.OnTop].set_value(obj2,True)
    
    def _set_under_kinematics(self,obj1:URDFObject,obj2:URDFObject):
        return obj1.states[object_states.Under].set_value(obj2,True)
    
    def _set_next_to_kinematics(self,obj1:URDFObject,obj2:URDFObject):
        return obj1.states[object_states.NextTo].set_value(obj2,True)
    
    # good for now
    def _set_in_side_magic(self,obj1:URDFObject,obj2:URDFObject):
        # target_center = get_aabb_center(obj2)
        # target_pos = tar_pos_for_new_aabb_center(obj1,target_center)
        # obj1.set_position(target_pos)
        # return obj1.states[object_states.Inside].get_value(obj2)
        obj1.set_position(obj2.get_position())
        return obj1.states[object_states.Inside].get_value(obj2)

    def _set_ontop_magic(self,obj1:URDFObject,obj2:URDFObject,offset=0.00):
        target_center = get_aabb_center(obj2)
        obj1_aabb=get_aabb(obj1)
        obj2_aabb=get_aabb(obj2)
        target_center[2] += 0.5 * obj1_aabb[2] + 0.5 *obj2_aabb[2] +offset
        target_pos = tar_pos_for_new_aabb_center(obj1,target_center)
        obj1.set_position(target_pos)
        return obj1.states[object_states.OnTop].get_value(obj2)

    def _set_under_magic(self,obj1:URDFObject,obj2:URDFObject,offset=0.00):
        target_center = get_aabb_center(obj2)
        obj1_aabb=get_aabb(obj1)
        obj2_aabb=get_aabb(obj2)
        target_center[2] -= 0.5 * obj1_aabb[2] + 0.5 *obj2_aabb[2] +offset
        target_pos = tar_pos_for_new_aabb_center(obj1,target_center)
        obj1.set_position(target_pos)
        return obj1.states[object_states.Under].get_value(obj2)

    def _set_next_to_magic(self,obj1:URDFObject,obj2:URDFObject):
        obj1_aabb=get_aabb(obj1)
        obj2_aabb=get_aabb(obj2)
        for i in [0,1]:
            for weight in [-1,1]:
                target_center = get_aabb_center(obj2)
                target_center[i] += weight*(0.5 * obj1_aabb[i] + 
                                            0.5 * obj2_aabb[i])
                target_pos = tar_pos_for_new_aabb_center(obj1,target_center)
                obj1.set_position(target_pos)
                if obj1.states[object_states.NextTo].get_value(obj2):
                    return True
        return False

    # ref: OcotoGibson
    def _set_robot_pos_for_obj_magic(self,obj:URDFObject):
        # get robot position according to object position
        obj_pos, obj_ori = obj.get_position_orientation()
        vec_standard = np.array([0, -1, 0])
        rotated_vec = Quaternion(obj_ori[[3, 0, 1, 2]]).rotate(vec_standard)
        bbox = get_aabb(obj)
        robot_pos = np.zeros(3)
        robot_pos[0] = obj_pos[0] + rotated_vec[0] * bbox[1] * 0.5 + rotated_vec[0]
        robot_pos[1] = obj_pos[1] + rotated_vec[1] * bbox[1] * 0.5 + rotated_vec[1]
        robot_pos[2] = 0.25

        self.robot.set_position(robot_pos)

    def _release_obj_magic(self,obj:URDFObject,offset=-0.01):
        target_center=get_aabb_center(obj)
        target_center[2] =offset
        target_pos = tar_pos_for_new_aabb_center(obj,target_center)
        obj.set_position(target_pos)

    def _set_in_hand_magic(self,obj:URDFObject,hand:str):
        weight=1 if hand=='right_hand' else -1
        target_pos = self.robot.get_position()
        target_pos[2] += self.robot.bounding_box[2]
        target_pos[2] +=0.2*weight
        obj.set_position(target_pos)
        return True
    

    ######################## Expose to action env###################
    def set_in_hand(self,obj:URDFObject,hand:str):
        return self._set_in_hand_magic(obj,hand)
    
    def set_inside(self,obj1:URDFObject,obj2:URDFObject):
        if self.using_kinematics:
            return self._set_inside_kinematics(obj1,obj2)
        return self._set_in_side_magic(obj1,obj2)
    
    def set_ontop(self,obj1:URDFObject,obj2:URDFObject):
        if self.using_kinematics:
            return self._set_ontop_kinematics(obj1,obj2)
        return self._set_ontop_magic(obj1,obj2)
    
    def set_under(self,obj1:URDFObject,obj2:URDFObject):
        if self.using_kinematics:
            return self._set_under_kinematics(obj1,obj2)
        return self._set_under_magic(obj1,obj2)

    def set_next_to(self,obj1:URDFObject,obj2:URDFObject):
        if self.using_kinematics:
            return self._set_next_to_kinematics(obj1,obj2)
        return self._set_next_to_magic(obj1,obj2)
    
    def set_robot_pos_for_obj(self,obj:URDFObject):
        return self._set_robot_pos_for_obj_magic(obj)
    
    def release_obj(self,obj:URDFObject):
        return self._release_obj_magic(obj)
    
    def set_in_hand(self,obj:URDFObject,hand:str):
        return self._set_in_hand_magic(obj,hand)
    