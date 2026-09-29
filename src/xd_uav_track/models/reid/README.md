# Vehicle ReID model

`vehicle-reid-0001.onnx` is the Open Model Zoo vehicle ReID model based on an
OmniScaleNet backbone. It accepts RGB `208x208` vehicle crops and produces a
512-dimensional descriptor compared with cosine similarity.

- Upstream model: `vehicle-reid-0001`
- Upstream file: `osnet_ain_x1_0_vehicle_reid.onnx`
- Source: https://storage.openvinotoolkit.org/repositories/open_model_zoo/public/2022.1/vehicle-reid-0001/osnet_ain_x1_0_vehicle_reid.onnx
- SHA-256: `4aaad3e5db648618b0df3d2ff21c61323985ff9e50194c3d2edd4fb87c92d91f`
- Upstream SHA-384: `0515ce72f653c39780d5b87dfed7255d396dd2b1e8b6e91fbaacdfad1da189166343157273c02f3b0fede3050ef7abb7`
- License: MIT; see `LICENSE.vehicle-reid-0001`

The published upstream VeRi-776 results apply to ordinary road vehicles, not
to this project's aerial tank imagery. The checkpoint is therefore the default
deep appearance frontend, while motion/world-coordinate gates remain
authoritative and the hybrid descriptor is retained as a model-load fallback.
Tank-specific ID-switch and long-occlusion validation is still required.
