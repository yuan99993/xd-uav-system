import unittest
import numpy as np
import cv2
from soft_tracks import SoftTrackLayer,candidates,compose_display

class SoftDisplayTests(unittest.TestCase):
 def setUp(self):
  rng=np.random.RandomState(7)
  self.image=np.zeros((240,320,3),np.uint8)
  self.image[60:120,80:150]=rng.randint(0,256,(60,70,3),dtype=np.uint8)
  self.rows=candidates([dict(box=[80,60,70,60],class_id=0,confidence=.9)],[])
 def test_pixels_follow_without_detection_and_gap_clears(self):
  layer=SoftTrackLayer(['car','tank','drone']);first=layer.step(self.image,1,0,self.rows)
  moved=cv2.warpAffine(self.image,np.float32([[1,0,9],[0,1,5]]),(320,240))
  second=layer.step(moved,1.05,0,[])
  self.assertEqual(len(second),1);self.assertEqual(first[0]['id'],second[0]['id'])
  np.testing.assert_allclose(second[0]['box'][:2],[89,65],atol=1)
  self.assertEqual(second[0]['last_perception_stamp'],1)
  self.assertEqual(layer.step(moved,1.5,0,[]),[])
 def test_late_detection_uses_previous_pixels(self):
  layer=SoftTrackLayer(['car']);layer.step(self.image,1,0,[])
  moved=cv2.warpAffine(self.image,np.float32([[1,0,9],[0,1,5]]),(320,240))
  result=layer.step(moved,1.05,0,[],self.rows)
  self.assertEqual(len(result),1);np.testing.assert_allclose(result[0]['box'][:2],[89,65],atol=1)
 def test_primary_is_not_soft_and_disappearance_removes_box(self):
  layer=SoftTrackLayer(['car']);rows=layer.step(self.image,1,0,self.rows)
  primary=dict(id=42,box=[80,60,70,60])
  self.assertEqual(compose_display(rows,42,[115,90,70,60],[primary]),[primary])
  self.assertEqual(layer.step(np.zeros_like(self.image),1.05,0,[]),[])
 def test_epoch_clears_and_candidates_are_perception_only(self):
  layer=SoftTrackLayer(['car']);layer.step(self.image,1,0,self.rows)
  self.assertEqual(layer.step(self.image,1.05,1,[]),[])
  self.assertEqual(candidates([dict(box=[1,2,20,20],class_id=0,confidence=.99,kind='manual_track')],[]),[])

 def test_repeated_rotation_does_not_compound_bounding_box(self):
  layer=SoftTrackLayer(['car']);box=[100.,100.,40.,20.]
  track=dict(quad=layer.corners(box),seed_size=box[2:])
  matrix=cv2.getRotationMatrix2D((120,110),2,1.)
  for _ in range(180):
   geometry=layer.project_geometry(track,matrix,(300,300,3))
   self.assertIsNotNone(geometry)
   bounded,quad=geometry;track['quad']=quad
   self.assertLessEqual(max(bounded[2:]),45.)
  np.testing.assert_allclose(bounded,box,atol=.02)
 def test_scale_drift_is_rejected_against_detection_size(self):
  layer=SoftTrackLayer(['car']);box=[100.,100.,40.,20.]
  track=dict(quad=layer.corners(box),seed_size=box[2:])
  matrix=cv2.getRotationMatrix2D((120,110),0,1.1)
  rejected=False
  for _ in range(10):
   result=layer.project_geometry(track,matrix,(300,300,3))
   if result is None:rejected=True;break
   track['quad']=result[1]
  self.assertTrue(rejected)
 def test_unconfirmed_visual_track_expires_even_with_matching_pixels(self):
  layer=SoftTrackLayer(['car']);layer.step(self.image,1,0,self.rows)
  for i in range(1,32):result=layer.step(self.image,1+i*.1,0,[])
  self.assertEqual(result,[])

if __name__=='__main__':unittest.main()
