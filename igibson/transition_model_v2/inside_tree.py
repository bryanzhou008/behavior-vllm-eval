from igibson.objects.articulated_object import URDFObject
from collections import defaultdict
import igibson.object_states as object_states

# To teleport the relationship of inside

class InsideNode:
    def __init__(self, obj: URDFObject):
        self.obj = obj
        self.children = {}
        self.parent = None

    def remove_node(self, node):
        self.children.pop(node.obj)

    def add_node(self, node):
        self.children[node.obj] = node
        node.parent = self

    def __hash__(self) -> int:
        return hash(self.obj)

class InsideTree:
    def __init__(self,obj_list):

        # convert a list of objects to a tree structure based on the inside relationship

        self.root = InsideNode(None)
        self.obj_to_node={}
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
                node=InsideNode(k)
                self.obj_to_node[k]=node
                cur_level.append(node)
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

        # build the tree, add links for each level
        for cur_level in hierarchy[1:]:
            new_leaf_nodes=[]
            for node_1 in leaf_nodes:
                picked_up=False
                for node_2 in cur_level:
                    if node_1.obj.states[object_states.Inside].get_value(node_2.obj):
                        node_2.add_node(node_1)
                        node_1.parent=node_2
                        picked_up=True
                        break
                if not picked_up:
                    new_leaf_nodes.append(node_1)
            leaf_nodes=new_leaf_nodes
            for node in cur_level:
                leaf_nodes.append(node)

        for node in leaf_nodes:
            self.root.add_node(node)
            node.parent=self.root

    def get_node(self,obj:URDFObject)->InsideNode:
        return self.obj_to_node[obj]
    
    def get_node_parent(self,obj:URDFObject)->InsideNode:
        return self.obj_to_node[obj].parent
    
    def place_inside(self,obj1:URDFObject,obj2:URDFObject):
        node1=self.get_node(obj1)
        node_1_parent=self.get_node_parent(obj1)
        node2=self.get_node(obj2)
        node_1_parent.remove_node(node1)
        node2.add_node(node1)
        node1.parent=node2

    def grasp(self,obj:URDFObject):
        node=self.get_node(obj)
        parent=node.parent
        parent.remove_node(node)
        node.parent=self.root
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
            for child in node.children.values():
                res+=print_tree(child,level+1)
            return res
        return print_tree(self.root,0)


    