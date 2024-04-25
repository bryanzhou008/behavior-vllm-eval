import numpy as np
from pyquaternion import Quaternion
from igibson.objects.articulated_object import URDFObject
import igibson.object_states as object_states
import pybullet as p
from igibson.utils.utils import restoreState

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

    def __init__(self,robot,simulator,using_kinematics=False):
        self.robot=robot
        self.robot.bounding_box=[0.5, 0.5, 1]
        self.using_kinematics=using_kinematics
        self.simulator=simulator

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

    # most tricky one
    def _set_ontop_magic(self,obj1:URDFObject,obj2:URDFObject,offset=0.00):
        # target_center = get_aabb_center(obj2)
        # obj1_aabb=get_aabb(obj1)
        # obj2_aabb=get_aabb(obj2)

        # x_min=target_center[0]-0.5*obj2_aabb[0]+0.5*obj1_aabb[0]
        # x_max=target_center[0]+0.5*obj2_aabb[0]-0.5*obj1_aabb[0]
        # y_min=target_center[1]-0.5*obj2_aabb[1]+0.5*obj1_aabb[1]
        # y_max=target_center[1]+0.5*obj2_aabb[1]-0.5*obj1_aabb[1]
        # cur_center=target_center.copy()
        # for x in np.linspace(x_min,x_max,x_steps):
        #     for y in np.linspace(y_min,y_max,y_steps):
        #         cur_center[0]=x
        #         cur_center[1]=y
        #         cur_center[2]= target_center[2]+0.5 * obj1_aabb[2] + 0.5 *obj2_aabb[2] +offset
        #         target_pos = tar_pos_for_new_aabb_center(obj1,target_center)
        #         obj1.set_position(target_pos)
        #         state=p.saveState()
        #         physics_timestep = p.getPhysicsEngineParameters()["fixedTimeStep"]
        #         for _ in range(int(falling_time / physics_timestep)):
        #             p.stepSimulation()
        #         obj1_pos,obj1_ori=obj1.get_position_orientation()
        #         if obj1.states[object_states.OnTop].get_value(obj2):
        #             print("Success")
        #             # obj2_pos,obj2_ori=obj2.get_position_orientation()
        #         else:
        #             print("Failed")
        #         restoreState(state)
        #         p.removeState(state)
        #         obj1.set_position_orientation(obj1_pos,obj1_ori)
        #         if obj1.states[object_states.OnTop].get_value(obj2):
        #             return True
        # return False
        target_center = get_aabb_center(obj2)
        obj1_aabb=get_aabb(obj1)
        obj2_aabb=get_aabb(obj2)
        target_center[2] += 0.5 * obj1_aabb[2] + 0.5 *obj2_aabb[2] +offset
        target_pos = tar_pos_for_new_aabb_center(obj1,target_center)
        obj1.set_position(target_pos)
        return obj1.states[object_states.OnTop].get_value(obj2)

    def _set_under_magic(self,obj1:URDFObject,obj2:URDFObject,offset=-0.01):
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

    def _release_obj_magic(self,obj:URDFObject,offset=0.01):
        target_center=get_aabb_center(obj)
        aabb=get_aabb(obj)
        target_center[2] =aabb[2]*0.5+offset
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
    