"""
Vehicle / tyre capability model for the Formula Student speed profiler.

This module describes the maximum longitudinal acceleration and braking
available to the vehicle for a given lateral acceleration.

this version uses two elliptical G-G envelopes:
Acceleration:
    (ax / AX_ACCEL_MAX)^2 + (ay / AY_MAX)^2 <= 1
Braking:
    (ax / AX_BRAKE_MAX)^2 + (ay / AY_MAX)^2 <= 1

Later, this module can be upgraded to use:
    - measured G-G data
    - speed-dependent G-G envelopes
    - aerodynamic downforce
    - powertrain limits
    - tyre load sensitivity
    - load transfer
    - Pacejka / Magic Formula tyres
    - a full vehicle dynamics model

remember to maintain tests for the functions in this file

CURRENT VEHICLE LIMITS
----------------------
Maximum speed:                70.0 m/s
Maximum forward acceleration:  6.0 m/s^2
Maximum braking deceleration: 10.0 m/s^2
Maximum lateral acceleration: 12.0 m/s^2
"""

from dataclasses import dataclass
import math

# DONT duplicate these values in external files, this should be the source of truth for vehicle limits

V_MAX = 70.0              # Maximum vehicle speed [m/s]
AX_ACCEL_MAX = 6.0        # Maximum longitudinal acceleration [m/s^2]
AX_BRAKE_MAX = 10.0       # Maximum braking deceleration magnitude [m/s^2]
AY_MAX = 12.0             # Maximum lateral acceleration magnitude [m/s^2]

# might possibly have to add a tolerance factor for floating point imprecision, that may make the function return infeasible
# even though the valuas are essentially equal to the limit




@dataclass(frozen=True)
class LongitudinalLimits:
    """
    Result type returned by get_longitudinal_limits(), class of 3 values.
    """
    ax_accel_max: float
    ax_brake_max: float
    feasible: bool



def get_longitudinal_limits(ay: float) -> LongitudinalLimits:
    """
    Calculate the available longitudinal acceleration and braking for
    a given lateral acceleration, with the lateral acceleration as input (sign doesnt matter for now).
    Returns LongitudinalLimits class with ax_accel_max (positive acc.), ax_brake_max (negative acc.), feasible 
    (whether requested lateral acceleration was within capability, else vehicle will slide?)


    Uses two elliptical G-G envelopes.
    Acceleration:
        (ax / AX_ACCEL_MAX)^2 + (ay / AY_MAX)^2 = 1

    Braking: (returned as negative)
        (ax / AX_BRAKE_MAX)^2 + (ay / AY_MAX)^2 = 1
    calculation uses:
        ax = AX_ACCEL/BRAKE_MAX *
             sqrt(1 - (ay / AY_MAX)^2)

    """

    # Check feasibility:

    ay_abs = abs(ay)

    if ay_abs > AY_MAX:
        # return 0's and infeasible case
        return LongitudinalLimits(
            ax_accel_max=0.0,
            ax_brake_max=0.0,
            feasible=False,
        )


    # Calculate the fraction of the lateral capability being used:
    lateral_ratio = ay_abs / AY_MAX


    # Calculate the remaining longitudinal capability:
    longitudinal_fraction = math.sqrt(
        max(0.0, 1.0 - lateral_ratio**2)
    )

    ax_accel_max = AX_ACCEL_MAX * longitudinal_fraction
    ax_brake_max = -AX_BRAKE_MAX * longitudinal_fraction


    # Return the results:
    return LongitudinalLimits(
        ax_accel_max=ax_accel_max,
        ax_brake_max=ax_brake_max,
        feasible=True,
    )

