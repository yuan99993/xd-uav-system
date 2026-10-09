import unittest
from xd_uav_track.perception_policy import admit_frame


class FrameAdmissionTest(unittest.TestCase):
    def test_jitter_does_not_accumulate(self):
        source = {}
        stamps = [10 + i/6. + (0.008 if i % 4 == 2 else 0.) for i in range(120)]
        accepted = [t for t in stamps if admit_frame(source, t, 3.)]
        self.assertGreaterEqual(len(accepted), 59)
        self.assertLessEqual(len(accepted), 61)
        self.assertLessEqual(max(b-a for a,b in zip(accepted, accepted[1:])), .51)
        previous = None
        legacy = []
        for stamp in stamps:
            if previous is None or stamp-previous >= 1./3.:
                previous = stamp
                legacy.append(stamp)
        self.assertGreater(len(accepted), len(legacy))

    def test_no_catchup_burst(self):
        source = {}
        self.assertTrue(admit_frame(source, 1., 3.))
        self.assertTrue(admit_frame(source, 10., 3.))
        self.assertFalse(admit_frame(source, 10.001, 3.))

    def test_rate_change_and_unlimited(self):
        source = {}
        self.assertTrue(admit_frame(source, 1., 3.))
        self.assertTrue(admit_frame(source, 1.1, 2.))
        self.assertFalse(admit_frame(source, 1.2, 2.))
        self.assertTrue(admit_frame(source, 1.2, 0.))
        self.assertTrue(admit_frame(source, 1.21, 2.))


if __name__ == '__main__':
    unittest.main()
