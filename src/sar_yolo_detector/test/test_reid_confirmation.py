import unittest
from sar_yolo_detector.reid_confirmation import ReIDConfirmation
class ConfirmationTest(unittest.TestCase):
    def test_two_fresh_forward_captures_required(self):
        c=ReIDConfirmation()
        self.assertFalse(c.update((10,10,20,20),1.,(10,0)))
        self.assertFalse(c.update((11,10,20,20),1.,(10,0)))
        self.assertTrue(c.update((11,10,20,20),1.1,(10,0)))
    def test_reverse_direction_gap_and_zero_direction_do_not_confirm(self):
        for box,stamp,velocity in [((9,10,20,20),1.1,(10,0)),((11,10,20,20),1.4,(10,0)),((10,10,20,20),1.1,(0,0)),((10,11,20,20),1.1,(10,0))]:
            c=ReIDConfirmation();c.update((10,10,20,20),1.,velocity)
            self.assertFalse(c.update(box,stamp,velocity))
