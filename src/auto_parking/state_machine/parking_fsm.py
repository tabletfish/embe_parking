from enum import Enum, auto


class ParkingState(Enum):
    MANUAL = auto()
    SEARCH_SLOT = auto()
    ALIGN_TO_ENTRY = auto()
    DRIVE_TO_ENTRY = auto()
    PARKING_MANEUVER = auto()
    FINAL_ALIGN = auto()
    SUCCESS = auto()
    FAIL = auto()


class ParkingFSM:
    def __init__(self, initial_state=ParkingState.MANUAL):
        self.state = initial_state

    def set_auto(self):
        self.state = ParkingState.SEARCH_SLOT

    def set_manual(self):
        self.state = ParkingState.MANUAL

    def update(self, slot_found=False, at_entry=False, parked=False, failed=False):
        if failed:
            self.state = ParkingState.FAIL
        elif self.state == ParkingState.SEARCH_SLOT and slot_found:
            self.state = ParkingState.ALIGN_TO_ENTRY
        elif self.state == ParkingState.ALIGN_TO_ENTRY:
            self.state = ParkingState.DRIVE_TO_ENTRY
        elif self.state == ParkingState.DRIVE_TO_ENTRY and at_entry:
            self.state = ParkingState.PARKING_MANEUVER
        elif self.state == ParkingState.PARKING_MANEUVER and parked:
            self.state = ParkingState.FINAL_ALIGN
        elif self.state == ParkingState.FINAL_ALIGN:
            self.state = ParkingState.SUCCESS
        return self.state
