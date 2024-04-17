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
Inside, 
NextTo, 
OnFloor, 
OnTop, 
Touching, 
Under, 

"""


class ActionPrimitives(IntEnum):
    NAVIGATE_TO = 0
    LEFT_GRASP = 1
    RIGHT_GRASP = 2
    LEFT_PLACE_ONTOP_RELEASE = 3
    RIGHT_PLACE_ONTOP_RELEASE = 4
    RIGHT_PLACE_ONTOP_NO_RELEASE = 5
    LEFT_PLACE_INSIDE_RELEASE = 6
    RIGHT_PLACE_INSIDE_RELEASE = 7
    LEFT_PLACE_INSIDE_NO_RELEASE = 8
    RIGHT_PLACE_INSIDE_NO_RELEASE = 9
    RIGHT_RELEASE = 10
    LEFT_RELEASE = 11
    PLACE_ON_TOP = 12
    PLACE_INSIDE = 13
    OPEN = 14
    CLOSE = 15
    BURN = 16
    COOK = 17
    CLEAN = 18
    FREEZE = 19
    UNFREEZE = 20
    SLICE = 21
    SOAK = 22
    DRY = 23
    STAIN = 24
    TOGGLE_ON = 25
    TOGGLE_OFF = 26
    UNCLEAN = 27
    UNSOAK = 28
   

    




