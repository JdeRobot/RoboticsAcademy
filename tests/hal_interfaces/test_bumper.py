import unittest
from hal_interfaces.general.bumper import (
    BumperData,
    contactsToBumperData,
    RIGHT_BUMPER,
    CENTER_BUMPER,
    LEFT_BUMPER,
)


# Simulate the structure of gazebo_msgs.msg.ContactsState
class ContactsState:
    def __init__(self, states=None):
        self.states = states if states is not None else []


class TestBumperLogic(unittest.TestCase):
    def test_bumperdata_str(self):
        bd = BumperData()
        bd.state = 1
        bd.bumper = LEFT_BUMPER
        s = str(bd)
        self.assertIn("state: 1", s)
        self.assertIn(f"bumper: {LEFT_BUMPER}", s)

    def test_contacts_to_bumperdata_none(self):
        # No contacts on any bumper
        contacts = [ContactsState([]), ContactsState([]), ContactsState([])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 0)
        self.assertEqual(bd.bumper, CENTER_BUMPER)

    def test_contacts_to_bumperdata_right(self):
        # Contact only on right bumper
        contacts = [ContactsState([object()]), ContactsState([]), ContactsState([])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, RIGHT_BUMPER)

    def test_contacts_to_bumperdata_center(self):
        # Contact only on center bumper
        contacts = [ContactsState([]), ContactsState([object()]), ContactsState([])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, CENTER_BUMPER)

    def test_contacts_to_bumperdata_left(self):
        # Contact only on left bumper
        contacts = [ContactsState([]), ContactsState([]), ContactsState([object()])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, LEFT_BUMPER)

    def test_contacts_to_bumperdata_multiple(self):
        # Contacts on multiple bumpers, should pick the first (right)
        contacts = [
            ContactsState([object()]),
            ContactsState([object()]),
            ContactsState([object()]),
        ]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, RIGHT_BUMPER)


if __name__ == "__main__":
    unittest.main()
