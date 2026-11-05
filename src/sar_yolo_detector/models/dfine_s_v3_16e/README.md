# D-FINE-S V3 (16e)

This directory contains the actual trained car/tank/drone weights used by the
four-mode UE tests, not a download pointer. `model.safetensors` SHA256:
`d4b408dd28f6e9d570304dce1bf342f8d1590193f05adca03f16dbc6ad1650bd`.

`manifest.json` pins the weights, resolved model configuration and native source
files at D-FINE commit `956d1709314c2c6a4df6f34de232054578a7449f`.
The package URI makes this deployment relocatable; the weight bytes are unchanged.
See [deployment instructions](../../docs/dfine_v3_deployment.md).
