import hashlib
from pathlib import Path
import tempfile
import unittest
import numpy as np
from sar_yolo_detector.roi_background_verifier import features,ROIBackgroundVerifier


class ROIVerifierContractTest(unittest.TestCase):
    def test_valid_small_roi_matches_serialized_feature_contract(self):
        frame=np.random.RandomState(17).randint(0,256,(60,90,3),dtype=np.uint8)
        result=features(frame,[10,10,18,20])
        self.assertEqual(result.shape,(378,))
        self.assertTrue(np.isfinite(result).all())

    def test_tiny_or_invalid_crop_supplies_no_veto_evidence(self):
        frame=np.zeros((40,40,3),np.uint8)
        for box in ([10,10,11,12],[-50,-40,-10,-5],[0,0,float('nan'),20]):
            self.assertIsNone(features(frame,box))

    def test_artifact_checksum_and_finite_contract_are_enforced(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'weights.npz'
            np.savez(path,mean=np.zeros(378),scale=np.ones(378),coefficients=np.zeros((379,4)))
            digest=hashlib.sha256(path.read_bytes()).hexdigest()
            with self.assertRaises(ValueError):ROIBackgroundVerifier(path,'0'*64)
            model=ROIBackgroundVerifier(path,digest)
            self.assertIsNone(model.background_margin(np.zeros((40,40,3),np.uint8),[0,0,1,1]))
            np.savez(path,mean=np.zeros(378),scale=np.zeros(378),coefficients=np.zeros((379,4)))
            digest=hashlib.sha256(path.read_bytes()).hexdigest()
            with self.assertRaises(ValueError):ROIBackgroundVerifier(path,digest)


if __name__=='__main__':unittest.main()
