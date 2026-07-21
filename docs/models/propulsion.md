# Propulsion-system reliability

The propulsion model evaluates how the number and placement of operational
motors affects vehicle reliability. Motor state is represented by a sequence in
which `1` means operational and `0` means failed.

## Supported configurations

| Configuration | Vehicle | Status-vector length |
| --- | --- | ---: |
| `PNPN` | Quadcopter | 4 |
| `PNPNPN` | Hexacopter, alternating rotation | 6 |
| `PPNNPN` | Hexacopter, grouped rotation | 6 |
| `PPNNPPNN` | Octocopter | 8 |

`P` and `N` describe the motor rotation direction around the airframe. Because
controllability after a motor failure depends on placement, two vehicles with
the same number of operational motors can occupy different Markov states.

## Example

```python
from SafeDrones.core.SafeDrones import SafeDrones

model = SafeDrones()

p_fail, mttf = model.Motor_Failure_Risk_Calc(
    MotorStatus=[0, 1, 1, 1, 1, 1],
    Motors_Configuration="PNPNPN",
    Motors_Lambda=0.001,
    time=5,
)
```

## Inputs

| Parameter | Description |
| --- | --- |
| `MotorStatus` | Ordered operational/failed state for every motor |
| `Motors_Configuration` | One of the supported rotation arrangements |
| `Motors_Lambda` | Propulsion-unit failure rate, including motor, driver, rotor, and propeller assumptions |
| `time` | Mission time at which failure probability is evaluated |

!!! important

    Keep `MotorStatus` in the same physical motor order used by the selected
    configuration. Reordering the same zeros and ones may change the result.

## Decision support

The output can support higher-level logic such as continuing the mission,
returning to the station, or initiating a safe landing. The decision thresholds
and safety policy remain the responsibility of the integrating system.
