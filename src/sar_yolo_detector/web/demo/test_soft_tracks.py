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
  self.assertEqual(layer.step(np.zeros_like(self.image),1.5,0,[]),[])
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

 def test_oversize_detection_rebuilds_on_current_frame(self):
  layer=SoftTrackLayer(['car']);gray=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
  layer.associate(self.image,gray,self.rows,1.0)
  first=next(iter(layer.tracks))
  larger=[dict(box=[70,50,115,90],class_id=0,confidence=.9)]
  layer.associate(self.image,gray,larger,1.05)
  self.assertNotIn(first,layer.tracks)
  self.assertEqual(len(layer.tracks),1)
  track=next(iter(layer.tracks.values()))
  self.assertEqual(track['box'],[70.,50.,115.,90.])
  self.assertEqual(track['last_perception_stamp'],1.05)
 def test_primary_switch_suppresses_new_primary_and_preserves_other_soft_track(self):
  one=dict(id=700000001,box=[20,20,20,20],center=[30,30])
  two=dict(id=700000002,box=[60,20,20,20],center=[70,30])
  a=compose_display([one,two],7,[30,30,20,20],[])
  b=compose_display([one,two],8,[70,30,20,20],[])
  self.assertEqual([o['id'] for o in a],[two['id']])
  self.assertEqual([o['id'] for o in b],[one['id']])

 def test_edge_clipping_uses_true_image_intersection(self):
  clip=SoftTrackLayer.clip
  self.assertEqual(clip([-4,10,14,12],(100,100,3)),[0.,10.,10.,12.])
  self.assertEqual(clip([10,-4,12,14],(100,100,3)),[10.,0.,12.,10.])
  self.assertEqual(clip([92,20,12,12],(100,100,3)),[92.,20.,8.,12.])
  self.assertIsNone(clip([-20,10,14,12],(100,100,3)))
  self.assertIsNone(clip([94,20,16,12],(100,100,3)))

 def test_prediction_is_brief_and_does_not_create_control_identity(self):
  layer=SoftTrackLayer(['car']);initial=layer.step(self.image,1.,0,self.rows)
  no_pixels=np.zeros_like(self.image)
  next_rows=layer.step(no_pixels,1.1,0,[])
  self.assertEqual(next_rows[0]['id'],initial[0]['id'])
  self.assertEqual(next_rows[0]['last_perception_stamp'],1.)
  self.assertEqual(layer.step(no_pixels,1.36,0,[]),[])
 def test_edge_center_controls_removal_and_partial_box_can_remain(self):
  clip=SoftTrackLayer.clip
  self.assertEqual(clip([-6,10,16,12],(100,100,3)),[0.,10.,10.,12.])
  self.assertIsNone(clip([-9,10,16,12],(100,100,3)))
  self.assertEqual(clip([92,10,12,12],(100,100,3)),[92.,10.,8.,12.])
  self.assertIsNone(clip([94,10,12,12],(100,100,3)))
 def test_limit_bounds_active_tracks(self):
  layer=SoftTrackLayer(['car']);layer.max_tracks=1
  rng=np.random.RandomState(2);image=rng.randint(0,256,(100,100,3),dtype=np.uint8)
  proposals=[dict(box=[10,10,20,20],class_id=0,confidence=.9),dict(box=[60,60,20,20],class_id=0,confidence=.8)]
  self.assertEqual(len(layer.step(image,1.,0,proposals)),1)

 def test_selection_preserves_nonprimary_until_detection_timeout(self):
  rng=np.random.RandomState(27)
  image=rng.randint(0,256,(200,300,3),dtype=np.uint8)
  layer=SoftTrackLayer(['car'])
  detections=[dict(box=[40,60,30,30],class_id=0,confidence=.9),
              dict(box=[180,60,30,30],class_id=0,confidence=.8)]
  before=layer.step(image,1.,0,detections)
  self.assertEqual(len(before),2)
  other=next(row for row in before if row['center'][0]>150)
  after=layer.step(image,1.05,0,[])
  visible=compose_display(after,15,[55,75,30,30],[])
  self.assertEqual([row['id'] for row in visible],[other['id']])
  self.assertEqual(visible[0]['last_perception_stamp'],1.)
  layer.step(image,1.25,0,[])
  self.assertEqual(layer.step(image,1.36,0,[]),[])

 def test_invalid_rebuild_keeps_existing_track_until_normal_timeout(self):
  layer=SoftTrackLayer(['car']);gray=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
  layer.step(self.image,1.0,0,self.rows)
  original=next(iter(layer.tracks))
  points=layer.points
  layer.points=lambda gray,box:None
  layer.associate(self.image,gray,[dict(box=[70,50,115,90],class_id=0,confidence=.9)],1.05)
  self.assertIn(original,layer.tracks)
  self.assertEqual(layer.tracks[original]['last_perception_stamp'],1.)
  layer.points=points
  self.assertEqual(len(layer.step(self.image,1.1,0,[])),1)
  self.assertEqual(layer.step(self.image,1.5,0,[]),[])

 def test_previous_capture_skips_repeated_candidates_but_accepts_late_additions(self):
  layer=SoftTrackLayer(['car'])
  rng=np.random.RandomState(33);image=rng.randint(0,256,(200,300,3),dtype=np.uint8)
  first=dict(box=[30,30,30,30],class_id=0,confidence=.9)
  late=dict(box=[150,30,30,30],class_id=0,confidence=.8)
  original=layer.points;calls=[]
  def counted(gray,box):
   calls.append(tuple(box));return original(gray,box)
  layer.points=counted
  layer.step(image,1.,0,[first]);calls.clear()
  result=layer.step(image,1.05,0,[],[first,late])
  self.assertEqual(len(result),2)
  self.assertEqual(calls,[(150.,30.,30.,30.)])

if __name__=='__main__':unittest.main()
