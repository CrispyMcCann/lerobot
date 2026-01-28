# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

LeRobot is a PyTorch library for real-world robotics by Hugging Face. It provides models, datasets, and tools for robot learning including imitation learning, reinforcement learning, and vision-language-action (VLA) models.

## Common Commands

### Installation (development)
```bash
conda create -y -n lerobot python=3.10 && conda activate lerobot
conda install ffmpeg -c conda-forge
pip install -e ".[dev,test]"
```

For simulation environments: `pip install -e ".[aloha,pusht]"`
For hardware: `pip install -e ".[feetech]"` or `pip install -e ".[dynamixel]"`

### Running Tests
```bash
# First ensure test artifacts are available
git lfs install && git lfs pull

# Run all tests
pytest -sv ./tests

# Run specific test file
pytest -sv tests/test_specific_feature.py
```

### Code Quality
```bash
# Install and run pre-commit hooks
pre-commit install
pre-commit run --all-files
```

Pre-commit runs: ruff (format + lint), typos, pyupgrade, bandit, mypy, and prettier for markdown.

### Training
```bash
lerobot-train \
  --policy.type=act \
  --dataset.repo_id=lerobot/aloha_mobile_cabinet
```

### Evaluation
```bash
lerobot-eval \
  --policy.path=path/to/checkpoint \
  --env.type=aloha \
  --eval.n_episodes=10
```

### End-to-End Tests (via Makefile)
```bash
make test-act-ete-train DEVICE=cuda  # or cpu
make test-act-ete-eval DEVICE=cuda
make test-end-to-end DEVICE=cuda     # runs all e2e tests
```

## Architecture

### Source Layout (`src/lerobot/`)

- **policies/**: Policy implementations (ACT, Diffusion, TDMPC, VQ-BeT, Pi0, SmolVLA, GR00T, etc.)
  - Each policy has `configuration_*.py`, `modeling_*.py`, and `processor_*.py`
  - Factory pattern in `factory.py` provides `get_policy_class()`, `make_policy()`, `make_policy_config()`
  - Base class: `PreTrainedPolicy` in `pretrained.py`

- **datasets/**: LeRobotDataset format (Parquet + MP4/images)
  - `LeRobotDataset` and `LeRobotDatasetMetadata` in `lerobot_dataset.py`
  - Dataset version: `CODEBASE_VERSION = "v3.0"`

- **robots/**: Robot hardware interfaces
  - Abstract base class: `Robot` in `robot.py` with `connect()`, `get_observation()`, `send_action()`, `disconnect()`
  - Implementations: SO100/101, Koch, LeKiwi, HopeJR, OMX, Reachy2, etc.

- **teleoperators/**: Teleoperation devices (gamepads, keyboards, phones)

- **motors/**: Motor drivers (Dynamixel, Feetech, Damiao)

- **cameras/**: Camera interfaces (OpenCV, Intel RealSense)

- **envs/**: Simulation environment wrappers (Aloha, PushT, LIBERO, MetaWorld)

- **configs/**: Configuration dataclasses using `draccus`
  - `TrainPipelineConfig` in `train.py`
  - `PreTrainedConfig` base for policies in `policies.py`

- **processor/**: Pre/post-processing pipelines for policies

- **scripts/**: CLI entry points (`lerobot-train`, `lerobot-eval`, `lerobot-record`, etc.)

### Configuration System

Uses `draccus` for dataclass-based configs with CLI parsing. Configs support:
- Loading from pretrained paths on Hub or local
- CLI overrides with dot notation (e.g., `--policy.device=cuda`)
- Nested configs for policy, dataset, env, optimizer, scheduler

### Key Patterns

1. **Policy Factory**: Use `get_policy_class(name)` and `make_policy(cfg, ds_meta)` from `policies/factory.py`

2. **Dataset Loading**: `LeRobotDataset(repo_id)` handles Hub download and video decoding

3. **Robot Interface**: All robots implement `Robot` ABC with `observation_features`, `action_features` properties

4. **Processors**: Each policy has pre/post-processors created via `make_*_pre_post_processors()` functions

### Adding New Components

When adding a new policy:
1. Create `configuration_*.py` and `modeling_*.py` in `policies/your_policy/`
2. Set required `name` and `config_class` attributes on the policy class
3. Update `available_policies` in `lerobot/__init__.py`
4. Update `tests/test_available.py`

When adding a new robot:
1. Implement `Robot` ABC in `robots/your_robot/`
2. Define `observation_features` and `action_features` properties
