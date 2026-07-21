# Getting started

## Requirements

SafeDrones is a Python package with numerical and symbolic modelling
dependencies. A dedicated virtual environment is recommended.

## Installation

=== "pip from GitHub"

    ```bash
    python -m pip install "git+https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones.git"
    ```

=== "local development"

    ```bash
    git clone https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones.git
    cd SafeDrones
    python -m pip install -e .
    ```

## First reliability assessment

Import the model class, configure the mission state, and calculate the combined
propulsion, battery, and processor risk:

```python
from SafeDrones.core.SafeDrones import SafeDrones

model = SafeDrones()
model.Set_Variables(
    MotorStatus=[1, 1, 1, 1, 1, 1],
    Motors_Configuration="PNPNPN",
    Motors_Lambda=0.001,
    BatteryLevel=80,
    time=100,
)

probability_of_failure, mttf = model.Drone_Risk_Calc()
print(float(probability_of_failure), float(mttf))
```

!!! note "Units must be consistent"

    Failure rates and mission time must use compatible units. For example, if
    the failure rate is expressed per hour, provide mission time in hours and
    interpret MTTF in hours.

## Assess one subsystem

You can call a model directly when only one subsystem is relevant:

```python
motor_risk, motor_mttf = model.Motor_Failure_Risk_Calc(
    MotorStatus=[1, 1, 1, 1, 1, 1],
    Motors_Configuration="PNPNPN",
    Motors_Lambda=0.001,
    time=10,
)
```

Each reliability method returns a tuple:

| Value | Meaning |
| --- | --- |
| `P_Fail` | Estimated probability of subsystem failure at the requested mission time |
| `MTTF` | Estimated mean time to failure |

## Explore the notebooks

The repository's [`example/`](https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones/tree/master/example)
directory contains Jupyter notebooks for complete and subsystem-focused
workflows.

## Build these docs locally

```bash
python -m pip install -r docs/requirements.txt
mkdocs serve
```

Open the local URL printed by MkDocs. Changes under `docs/` reload automatically.
