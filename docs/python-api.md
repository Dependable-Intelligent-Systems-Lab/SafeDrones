# Python API

## Import

```python
from SafeDrones.core.SafeDrones import SafeDrones
```

Create one instance and either configure defaults with `Set_Variables` or pass
arguments to individual model methods.

## `Set_Variables`

Stores the current vehicle and mission state on the model instance. Any omitted
argument to a later model call uses the stored value where supported.

```python
model = SafeDrones()
model.Set_Variables(
    MotorStatus=[1, 1, 1, 1, 1, 1],
    Motors_Configuration="PNPNPN",
    Motors_Lambda=0.001,
    Batt_Lambda=0.001,
    BatteryLevel=80,
    time=100,
)
```

## Methods

### `Motor_Failure_Risk_Calc`

```python
Motor_Failure_Risk_Calc(
    MotorStatus=None,
    Motors_Configuration=None,
    Motors_Lambda=None,
    time=None,
)
```

Returns `(P_Fail, MTTF)` for the selected propulsion configuration.

### `Battery_Failure_Risk_Calc`

Numerical battery model intended for faster evaluation. Returns
`(P_Fail, MTTF)`.

### `Battery_Failure_Risk_Calc_precise`

Symbolic version of the battery model. It follows the same input structure but
may take substantially longer.

### `Chip_MTTF_Model`

Temperature- and utilization-adjusted processor model. Returns
`(P_Fail, MTTF)`.

### `GPS_Failure_Risk_Calc`

Binomial `k`-out-of-`n` satellite-availability model. Returns
`(P_Fail, MTTF)`.

### `calculate_collision_risk`

Returns `(danger_zone_risk, collision_zone_risk)` for two equally sampled
trajectories.

### `Drone_Risk_Calc`

Uses the values stored on the model to combine propulsion, battery, and
processor estimates into `(P_Fail_Total, MTTF_Total)`.

## Result conversion

Some symbolic methods return SymPy numeric objects. Convert them when passing
results to systems that expect native Python numbers:

```python
p_fail, mttf = model.Drone_Risk_Calc()
payload = {"probability_of_failure": float(p_fail), "mttf": float(mttf)}
```

!!! warning

    The current API does limited validation. Validate ranges, status-vector
    lengths, units, and supported configurations in the integrating application.
