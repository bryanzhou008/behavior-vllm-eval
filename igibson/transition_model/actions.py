from enum import IntEnum


"""
States list

Burnt, --> Burn
CleaningTool, 
HeatSourceOrSink, 
Cooked, --> Cook
Dusty,  --> True->False: Clean
Frozen, --> Freeze/Unfreeze
Open,   
Sliced, --> False->True: Slice
Slicer, 
Soaked, --> Soak/Dry
Stained, --> True->False: Stain
ToggledOn, --> Toggle
WaterSource, 
InFOVOfRobot, 
InHandOfRobot, 
InReachOfRobot, 
InSameRoomAsRobot, 
InRoom, 
Inside, 
NextTo, 
OnFloor, 
OnTop, 
Touching, 
Under 

"""
class ActionPrimitives(IntEnum):
    NAVIGATE_TO = 0
    LEFT_GRASP = 1
    RIGHT_GRASP = 2
    LEFT_PLACE_ONTOP = 3
    RIGHT_PLACE_ONTOP = 4
    LEFT_PLACE_INSIDE = 5
    RIGHT_PLACE_INSIDE = 6
    OPEN = 7
    CLOSE = 8
    BURN=9
    COOK=10
    CLEAN=11
    FREEZE=12
    UNFREEZE=13
    SLICE=14
    SOAK=15
    DRY=16
    STAIN=17
    TOGGLE=18
