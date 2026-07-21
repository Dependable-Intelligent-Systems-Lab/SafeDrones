# Reliability models

SafeDrones combines several subsystem models behind one Python class. Each model
accepts the latest observed state and returns a failure probability and an MTTF
estimate.

## Model map

| Model | Main inputs | Method |
| --- | --- | --- |
| Propulsion | Motor status, rotor configuration, failure rate, mission time | `Motor_Failure_Risk_Calc` |
| Battery | Charge level, failure/degradation rates, charge/discharge rates, time | `Battery_Failure_Risk_Calc` |
| Processor | Reference MTTF, reference/actual temperature, utilization, Weibull beta | `Chip_MTTF_Model` |
| GPS | Visible satellites, required satellites, single-link failure rate, time | `GPS_Failure_Risk_Calc` |
| Collision | Two sampled trajectories, danger and collision thresholds | `calculate_collision_risk` |
| Combined drone | Current configured propulsion, battery, and processor state | `Drone_Risk_Calc` |

## Combined risk

`Drone_Risk_Calc` treats propulsion, battery, and processor failure as
independent for its combined probability:

```text
P(total failure) = 1 - (1 - Pmotor)(1 - Pbattery)(1 - Pprocessor)
```

The combined MTTF is the minimum of the three subsystem estimates.

!!! warning "Model assumptions matter"

    Independence and constant-rate assumptions are modelling choices, not
    guarantees about a physical aircraft. Validate input rates and thresholds
    against the vehicle, operational environment, and applicable assurance
    process before using results in decisions.

## Selecting a model

- Use the [propulsion model](propulsion.md) when rotor placement and individual
  motor state determine controllability after failures.
- Use the [component models](components.md) for battery, processor, or GPS
  reliability.
- Use combined drone risk for a compact mission-level indicator after all input
  values have been configured consistently.
