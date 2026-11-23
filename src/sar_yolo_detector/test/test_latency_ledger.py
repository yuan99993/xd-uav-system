import unittest
from sar_yolo_detector.latency_ledger import breakdown,join_controls,BOUNDARIES
class LedgerTest(unittest.TestCase):
    def test_additive_closure_and_reject_bad_clocks(self):
        t=dict(zip(BOUNDARIES,[10.,10.017,10.028,10.030,10.044,10.047]))
        d=breakdown(t);self.assertTrue(d['valid']);self.assertAlmostEqual(d['total_ms'],47.)
        self.assertAlmostEqual(d['closure_error_ms'],0.)
        self.assertFalse(breakdown({})['valid'])
        t['vision_complete_wall']=9.;self.assertFalse(breakdown(t)['valid'])
    def test_join_uses_capture_identity_and_causally_prior_revision(self):
        controls=[dict(capture_stamp=10.,capture_ns=10000000000,control_output_wall=10.06)]
        times=dict(zip(BOUNDARIES[:-1],[10.,10.017,10.028,10.030,10.044]))
        events=[dict(capture_ns=10000000000,node_state_publish_wall=10.045,timestamps=times),
                dict(capture_ns=10000000000,node_state_publish_wall=10.070,timestamps={}),
                dict(capture_ns=999,node_state_publish_wall=10.05,timestamps={})]
        row=join_controls(controls,events)[0]
        self.assertTrue(row['latency_ledger']['valid']);self.assertEqual(row['node_timing']['node_state_publish_wall'],10.045)
        self.assertFalse(join_controls(controls,[])[0]['latency_ledger']['valid'])

    def test_revision_match_uses_serialized_processing_signature(self):
        t=dict(zip(BOUNDARIES[:-1],[10.,10.01,10.02,10.03,10.04]))
        controls=[dict(capture_stamp=10.,capture_ns=10,control_output_wall=10.08,frame_processing_ms=22.)]
        e1=dict(capture_ns=10,node_state_publish_wall=10.05,timestamps=t,state_processing_ms=22.)
        e2=dict(capture_ns=10,node_state_publish_wall=10.06,timestamps=t,state_processing_ms=23.)
        result=join_controls(controls,[e1,e2])[0]
        self.assertEqual(result['node_timing']['state_processing_ms'],22.)
