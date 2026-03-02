# Model Selector

Download and swap openpilot driving and driver monitoring models. Discovers new models from the [openpilot](https://github.com/commaai/openpilot) repository, downloads ONNX files via Git LFS, and handles device-specific PKL compilation caching.

## How It Works

openpilot ships with one driving model and one driver monitoring (DM) model. Model Selector lets you switch between multiple versions — useful for testing new releases or rolling back if a model misbehaves.

**Two independent model types:**

| Type | ONNX files | Storage |
|------|-----------|---------|
| Driving | `driving_vision.onnx` + `driving_policy.onnx` | `/data/models/driving/` |
| Driver Monitoring | `dmonitoring_model.onnx` | `/data/models/dm/` |

**ONNX + PKL caching:**

Models are stored as portable ONNX files downloaded from GitHub. On first boot after a swap, openpilot compiles ONNX to device-specific tinygrad PKL files (~60s on C3). Model Selector caches compiled PKL files per model, so subsequent swaps to a previously used model skip compilation entirely. PKL cache is invalidated automatically when the tinygrad version changes.

## For Users

Use [Connect on Device](https://github.com/catpilot-dev/connect) to browse, download, and swap models from the web UI — no SSH required.

## For Developers

### Model registry

The registry at `/data/models/model_registry.json` tracks available models with their GitHub commit hashes, dates, and PR references. Update it from GitHub:

```bash
python model_download.py update-registry
```

### Download a model

```bash
# List available models
python model_download.py list
python model_download.py list --type driving

# Download a specific model
python model_download.py download --type driving cool_people_3c957c6

# Add a model from a GitHub PR
python model_download.py add-from-pr 36849

# Check for new models not yet installed
python model_download.py check-updates
```

### Swap the active model

```bash
# List installed models
python model_swapper.py --type driving list

# Show active model
python model_swapper.py --type driving active

# Swap (requires reboot)
python model_swapper.py --type driving swap cool_people_3c957c6

# Verify ONNX files and PKL cache
python model_swapper.py --type driving verify cool_people_3c957c6

# Delete an installed model
python model_swapper.py --type driving delete old_model_abc1234
```

### File layout on device

```
/data/models/
├── model_registry.json          # Available models catalog
├── active_driving_model          # JSON tracker (current driving model ID)
├── active_dm_model               # JSON tracker (current DM model ID)
├── driving/
│   └── cool_people_3c957c6/
│       ├── model_info.json       # Name, commit, date, description
│       ├── driving_vision.onnx   # Portable (from GitHub)
│       ├── driving_policy.onnx   # Portable (from GitHub)
│       ├── driving_vision_tinygrad.pkl   # Compiled (device-specific cache)
│       ├── driving_policy_tinygrad.pkl   # Compiled (device-specific cache)
│       └── .tinygrad_commit      # tinygrad version that built the PKL
└── dm/
    └── medium_fanta_cc8f6ea/
        ├── model_info.json
        ├── dmonitoring_model.onnx
        └── dmonitoring_model_tinygrad.pkl
```

### Compatibility filtering

Driving models older than the `desire_pulse` transition (August 27, 2025) are incompatible with openpilot v0.10.1+ and are flagged during download. The registry auto-update also excludes reverted models.
