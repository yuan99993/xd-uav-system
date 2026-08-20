# SeaDronesSee maritime person-overboard model

This directory contains the stable artifacts produced by the
`maritime_seadronessee` training profile.  The model uses the official
SeaDronesSee v2 category order:

0. `swimmer`
1. `boat`
2. `jetski`
3. `life_saving_appliances`
4. `buoy`

`rescue_profile.launch profile:=maritime_person` loads
`maritime_person_fp16.engine` by default.  The package also keeps the
PyTorch checkpoint and ONNX export for retraining or a non-TensorRT backend.
`training_metadata.json` records the dataset run and validation metrics.
