# Battery, processor, GPS, and collision models

## Battery

The approximate battery method uses a numerical matrix exponential and is the
faster choice for repeated evaluation:

```python
p_fail, mttf = model.Battery_Failure_Risk_Calc(
    BatteryLevel=80,
    time=10,
    Lambda=0.001,
    alpha=0.008,
    beta=0.007,
    Battery_degradation_rate=0.0064,
)
```

`Battery_Failure_Risk_Calc_precise` evaluates the corresponding symbolic model
and can take longer.

| Input | Meaning |
| --- | --- |
| `BatteryLevel` | Charge percentage from 0 to 100 |
| `Lambda` | Battery-system component failure rate |
| `alpha` / `beta` | Charge and discharge transition rates |
| `Battery_degradation_rate` | Rate of transition to a degraded charge state |

At 25% charge or below, the implementation reports failure with zero MTTF.

## Processor

The processor model applies an Arrhenius acceleration factor to a reference
MTTF, adjusts for utilization, and evaluates failure using a Weibull model.

```python
p_fail, mttf = model.Chip_MTTF_Model(
    MTTFref=400,
    Tr=303.15,
    Ta=323.15,
    u=1,
    b=1,
    time=10,
)
```

Use absolute temperature consistently when applying an Arrhenius relationship.

## GPS

The GPS method models the connection as a `k`-out-of-`n` system:

```python
p_fail, mttf = model.GPS_Failure_Risk_Calc(
    SatStatus=22,
    time=10,
    Lambda=0.001,
    MaxSat=29,
    MinSat=17,
)
```

If fewer than `MinSat` satellites are available, the implementation returns
failure with zero MTTF.

## Collision and danger zones

`calculate_collision_risk` compares synchronized samples from two trajectories.
It returns the fraction of samples inside the danger and collision thresholds:

```python
uav_1 = [(0, 0), (1, 1), (2, 2)]
uav_2 = [(5, 5), (1.5, 1.5), (2.1, 2.1)]

danger_risk, collision_risk = model.calculate_collision_risk(
    uav_1,
    uav_2,
    danger_threshold=2.0,
    collision_threshold=0.5,
)
```

Both trajectories must contain the same number of samples and use the same
coordinate system and distance unit as the thresholds.
