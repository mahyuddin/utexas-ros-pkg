#Possbile devices that can be run on the system
#This should include displays and interfaces as well
uint8 BWI_DEVICE_ROBOT = 0
uint8 BWI_DEVICE_CAMERA = 1

#Possible applications (from which information will have to be extracted)
# Robot applications
uint8 BWI_APP_ROBOT_NAVIGATOR = 0
uint8 BWI_APP_ROBOT_MAPPER = 1
uint8 BWI_APP_ROBOT_TELEOPERATOR = 2

# Person detection
uint8 BWI_APP_PERSON_DETECTOR = 20
uint8 BWI_APP_PERSON_AGGREGATOR = 21

# Multi robot navigator
uint8 BWI_APP_COMMANDER_NAVIGATOR = 40 

#Possible machine types (can be combined)

# machines connected to devices
uint32 BWI_ROBOT_MACHINE = 1
uint32 BWI_CAMERA_MACHINE = 2

# machines for data aggregation
uint32 BWI_COMMANDER_MACHINE = 2048
