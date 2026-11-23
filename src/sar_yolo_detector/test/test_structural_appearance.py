import unittest
import numpy as np
from sar_yolo_detector.box_fusion import appearance

class StructuralAppearanceTest(unittest.TestCase):
    def test_small_texture_has_quality_but_flat_background_does_not(self):
        rng=np.random.RandomState(92)
        gray=rng.randint(0,255,(16,16),dtype=np.uint8)
        image=np.repeat(gray[:,:,None],3,axis=2)
        old,old_quality=appearance(image,(0,0,16,16))
        feature,quality=appearance(image,(0,0,16,16),structural=True)
        self.assertEqual(len(feature),len(old))
        self.assertGreater(quality,old_quality)
        self.assertAlmostEqual(float(np.linalg.norm(feature)),1.,places=6)
        _,blank=appearance(np.full_like(image,100),(0,0,16,16),structural=True)
        self.assertEqual(blank,0.)

    def test_grayscale_structure_is_distinguished(self):
        a=np.full((24,24,3),100,np.uint8)
        a[:,8:16]=200
        b=np.transpose(a,(1,0,2)).copy()
        first,_=appearance(a,(0,0,24,24),structural=True)
        second,_=appearance(b,(0,0,24,24),structural=True)
        self.assertLess(float(first@second),.8)
