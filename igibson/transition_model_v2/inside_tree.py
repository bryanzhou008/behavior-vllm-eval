from igibson.objects.articulated_object import URDFObject
from collections import defaultdict
import igibson.object_states as object_states

class InsideNode:
    def __init__(self, obj: URDFObject):
        self.obj = obj
        self.inside = {}

    def remove_node(self, node):
        self.inside.pop(node.obj)

    def add_node(self, node):
        self.inside[node.obj] = node

    def __hash__(self) -> int:
        return hash(self.obj)

class InsideTree:
    def __init__(self,obj_list):

        # convert a list of objects to a tree structure based on the inside relationship

        self.root = InsideNode(None)

        tmp_dict={}

        for obj1 in obj_list:
            tmp_dict[obj1]=set()
            for obj2 in obj_list:
                if obj1 == obj2 or not isinstance(obj1, URDFObject) or not isinstance(obj2, URDFObject):
                    continue
                if obj2.states[object_states.Inside].get_value(obj1):
                    tmp_dict[obj1].add(obj2)

        
        # get the hierarchy of the tree
        hierarchy = []
        i=0
        while len(tmp_dict)>0:
            cur_level = []
            for k,v in tmp_dict.items():
                if len(v)!=0:
                    continue
                cur_level.append(InsideNode(k))
            for v in tmp_dict.values():
                for node in cur_level:
                    if node.obj in v:
                        v.remove(node.obj)
            for node in cur_level:
                tmp_dict.pop(node.obj)
            hierarchy.append(cur_level) 
        
       
        if len(hierarchy)==0:
            return
        
        leaf_nodes=[node for node in hierarchy[0]]

        for cur_level in hierarchy[1:]:
            new_leaf_nodes=[]
            for node_1 in leaf_nodes:
                picked_up=False
                for node_2 in cur_level:
                    if node_1.obj.states[object_states.Inside].get_value(node_2.obj):
                        node_2.add_node(node_1)
                        picked_up=True
                        break
                if not picked_up:
                    new_leaf_nodes.append(node_1)
            leaf_nodes=new_leaf_nodes
            for node in cur_level:
                leaf_nodes.append(node)

        for node in leaf_nodes:
            self.root.add_node(node)

    def get_node(self,obj:URDFObject):
        def get_node_helper(node:InsideNode):
            if node.obj==obj:
                return node
            for child in node.inside.values():
                res=get_node_helper(child)
                if res is not None:
                    return res
            return None
        return get_node_helper(self.root)
    
    def get_node_parent(self,obj:URDFObject):
        def get_node_parent_helper(node:InsideNode):
            for child in node.inside.values():
                if child.obj==obj:
                    return node
                res=get_node_parent_helper(child)
                if res is not None:
                    return res
            return None
        return get_node_parent_helper(self.root)
    
    def place_inside(self,obj1:URDFObject,obj2:URDFObject):
        node1=self.get_node(obj1)
        node_1_parent=self.get_node_parent(obj1)
        node2=self.get_node(obj2)
        node_1_parent.remove_node(node1)
        node2.add_node(node1)

    def grasp(self,obj:URDFObject):
        node=self.get_node(obj)
        node_parent=self.get_node_parent(obj)
        node_parent.remove_node(node)
        self.root.add_node(node)

    def __str__(self) -> str:
        def print_tree(node:InsideNode,level):
            res=""
            for i in range(level):
                res+="  "
            
            if node.obj is None:
                res+="root\n"
            else:
                res+=str(node.obj.name)+'\n'
            for child in node.inside.values():
                res+=print_tree(child,level+1)
            return res
        return print_tree(self.root,0)


    