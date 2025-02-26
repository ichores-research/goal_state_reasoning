import enum

class Task(enum.Enum):
    GET_OBJECT_NAMES = "get_object_names"
    GET_OBJECTS = "get_objects"
    GET_POINTING_SEQUENCE = "get_pointing_sequence"
    PICK_OBJECT = "pick_object"
    PLACE_OBJECT = "place_object"
    RELEASE_PICKED_OBJECT = "release_picked_object"

