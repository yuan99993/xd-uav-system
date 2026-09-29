# XD vehicle train7 candidate archive

This is the train7 model that was tested against `predict7`, `predict4`, and
`car.rar`. It was not kept as the active SmartTracker model because its
positive-set coverage was lower than the previous train model on all tested
datasets.

- Model: `xd_vehicle_train7.pt`
- SHA-256: `8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54`
- Active model after rollback: `../xd_vehicle_latest/xd_vehicle_latest.pt`

FP16 deployment artifact for this host:

- Engine: `xd_vehicle_train7.engine` (TensorRT 10.1, CUDA 12.4, RTX 3050)
- Engine SHA-256: `88ed19419dcefdf64f6c90f31aa7961ef906c31d3581633afba69fa586bc850f`
- Input: static `1x3x640x640`, FP16, builder optimization level 1
- Provenance: `xd_vehicle_train7.engine.json`

The engine is hardware/runtime specific and is not a portable replacement for
the `.pt` source. FP16 conversion preserves the model architecture and class
contract but has not itself been accuracy-regression tested on the benchmark
videos; the earlier train7 dataset coverage caveat above still applies.
