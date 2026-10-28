import unittest
from hal_interfaces.general.bumper import (
    BumperData,
    contactsToBumperData,
    RIGHT_BUMPER,
    CENTER_BUMPER,
    LEFT_BUMPER,
)


# Simulate the structure of ros_gz_interfaces.msg.Contacts
class MockContacts:
    def __init__(self, contacts=None):
        self.contacts = contacts if contacts is not None else []


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
        contacts = [MockContacts([]), MockContacts([]), MockContacts([])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 0)
        self.assertEqual(bd.bumper, CENTER_BUMPER)

    def test_contacts_to_bumperdata_right(self):
        # Contact only on right bumper
        contacts = [MockContacts([object()]), MockContacts([]), MockContacts([])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, RIGHT_BUMPER)

    def test_contacts_to_bumperdata_center(self):
        # Contact only on center bumper
        contacts = [MockContacts([]), MockContacts([object()]), MockContacts([])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, CENTER_BUMPER)

    def test_contacts_to_bumperdata_left(self):
        # Contact only on left bumper
        contacts = [MockContacts([]), MockContacts([]), MockContacts([object()])]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, LEFT_BUMPER)

    def test_contacts_to_bumperdata_multiple(self):
        # Contacts on multiple bumpers, should pick the first (right)
        contacts = [
            MockContacts([object()]),
            MockContacts([object()]),
            MockContacts([object()]),
        ]
        bd = contactsToBumperData(contacts)
        self.assertEqual(bd.state, 1)
        self.assertEqual(bd.bumper, RIGHT_BUMPER)


if __name__ == "__main__":
    unittest.main()
