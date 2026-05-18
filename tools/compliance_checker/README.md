# ROS OpenUSD Interoperability Compliance Checker

> [!NOTE]
> This project is a proof of concept developed using agentic coding.

Validate OpenUSD assets against REP-0158 interoperability rules for simulation, ROS integration, and export portability.

> [!TIP]
> Run the checker on any asset without cloning\*:
>
> ```bash
> uvx --from ros-openusd-compliance-checker usd-check <asset.usd>
> ```
>
> \* uv must be installed first:
>
> ```bash
> curl -LsSf https://astral.sh/uv/install.sh | sh
> ```

## Installation

```bash
pip install ros-openusd-compliance-checker
```

## Usage

```bash
usd-check [OPTIONS] ASSET
```

By default only the **core** checks run (REP §1 and §2: units, structure,
physics, and the five base ROS schemas — `RosContextAPI`, `RosTopicAPI`,
`RosServiceAPI`, `RosActionAPI`, `RosFrameAPI`).

| Flag           | What it adds                                                  |
| -------------- | ------------------------------------------------------------- |
| _(default)_    | §1 units/structure/physics + §2 core ROS schemas              |
| `--export`     | §3 export/conversion checks (mesh, texture, material; slower) |

```bash
# Core checks only (default)
usd-check robot.usda

# Core + export checks
usd-check robot.usda --export
```

## Developer Setup

### Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Install repository

```bash
git clone https://github.com/ros-simulation/openusd-schemas.git
cd openusd-schemas/tools/compliance_checker
```

### Install dependencies

```bash
uv sync --all-packages
```

### Run tests

```bash
uv run pytest
```

### Run pre-commit

```bash
uv run pre-commit
```
