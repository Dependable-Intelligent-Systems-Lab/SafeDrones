# SafeDrones

<div class="hero" markdown>
  <img src="assets/images/logo.png" alt="SafeDrones logo" class="hero__logo">

  **Real-time reliability and safety evaluation for multicopters and eVTOL aircraft.**

  SafeDrones turns current vehicle health, configuration, and mission conditions
  into interpretable failure probability and mean-time-to-failure estimates.

  [Get started](getting-started.md){ .md-button .md-button--primary }
  [View on GitHub](https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones){ .md-button }
</div>

[![License](https://img.shields.io/github/license/Dependable-Intelligent-Systems-Lab/SafeDrones?color=yellow)](https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones/blob/master/LICENSE)
[![Python](https://img.shields.io/badge/Python-%3E%3D3.5-blue)](https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones)
[![MATLAB](https://img.shields.io/badge/MATLAB-supported-orange)](matlab.md)
[![Documentation](https://img.shields.io/badge/docs-Read%20the%20Docs-teal)](https://safedrones.readthedocs.io/)

## What can SafeDrones evaluate?

<div class="grid cards" markdown>

-   **Propulsion reliability**

    ---

    Estimate failure probability and MTTF for quadcopter, hexacopter, and
    octocopter motor arrangements using configuration-aware Markov models.

    [Propulsion model →](models/propulsion.md)

-   **Battery health**

    ---

    Model battery failure from charge level, degradation, charge/discharge
    rates, component failure rate, and mission duration.

    [Component models →](models/components.md)

-   **Processor reliability**

    ---

    Account for temperature, utilization, and Weibull lifetime behaviour using
    an Arrhenius-based processor model.

    [Component models →](models/components.md)

-   **GPS and collision risk**

    ---

    Evaluate satellite-availability reliability and measure trajectory samples
    that enter configured danger or collision zones.

    [Python API →](python-api.md)

</div>

## How it fits into an autonomous system

```text
Health monitoring and diagnosis
              ↓
Vehicle state + mission time + failure rates
              ↓
        SafeDrones models
              ↓
Probability of failure + mean time to failure
              ↓
Continue mission · return to base · land safely
```

!!! tip

    Use SafeDrones alongside a fault-detection and diagnosis system. The models
    consume observations such as motor status, battery charge, temperature, and
    visible GPS satellites; they do not replace the sensors or diagnosis layer.

## Supported implementations

| Interface | Best for | Included resources |
| --- | --- | --- |
| Python | Integration, notebooks, automated analysis | Package source and example notebooks |
| MATLAB | Model exploration and engineering workflows | Propulsion, battery, and GPS scripts |

## Start here

1. [Install SafeDrones and run a first assessment](getting-started.md).
2. [Choose the reliability model](models/index.md) for your subsystem.
3. Review the [Python API](python-api.md) or [MATLAB guide](matlab.md).
4. Cite the relevant work from the [research page](research.md).
