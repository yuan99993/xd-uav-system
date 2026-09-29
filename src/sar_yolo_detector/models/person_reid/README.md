# Deep person ReID model

`osnet_x0_25_market1501.pt` is an OSNet-x0.25 person re-identification
checkpoint trained with the Torchreid model zoo on Market-1501.  It is used
only to produce an appearance embedding for a person that has already been
detected; it is not a detector and cannot recover a person missed by YOLO.

The SmartTracker wrapper verifies this exact artifact before loading it:

- SHA-256: `c0ff09177a21417a19bc73bbf5af4eb5d2b097c2074ed35e67819ee6cd93612c`
- Input: BGR crop, resized to 256x128 (height x width), ImageNet normalization
- Runtime: `torchreid==0.2.5`, PyTorch and torchvision

The checkpoint is a generic pedestrian ReID model.  It must be validated and,
where necessary, fine-tuned with the project’s NITC, disaster, thermal and
maritime data before identity associations are used in an operational rescue
workflow.  It does not provide cross-UAV identity guarantees.

Torchreid and the OSNet implementation are distributed under the MIT license;
retain the upstream attribution and license records in
`THIRD_PARTY_NOTICES.md` when redistributing this artifact.
