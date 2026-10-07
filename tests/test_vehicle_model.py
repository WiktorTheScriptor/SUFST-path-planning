"""
Pytest Tests for vehicle_model.py.

These tests verify:
    - The G-G envelope calculations are correct.
    - Acceleration and braking limits are asymmetric as intended.
    - Positive and negative lateral acceleration behave identically.
    - The maximum lateral acceleration is handled correctly.
    - Lateral acceleration beyond the vehicle limit is rejected.
    - The returned LongitudinalLimits object has the expected structure.
"""

import math

from speed_profiler.vehicle_model import (
    AX_ACCEL_MAX,
    AX_BRAKE_MAX,
    AY_MAX,
    LongitudinalLimits,
    get_longitudinal_limits,
)


def test_zero_lateral_acceleration():
    """
    With zero lateral acceleration, the vehicle should have its full
    longitudinal acceleration and braking capability available.
    """

    result = get_longitudinal_limits(0.0)

    assert result.feasible is True
    assert result.ax_accel_max == AX_ACCEL_MAX
    assert result.ax_brake_max == -AX_BRAKE_MAX


def test_half_lateral_acceleration():
    """
    At half the maximum lateral acceleration, verify that the remaining
    longitudinal capability follows the elliptical G-G equation.
    """

    ay = AY_MAX / 2.0

    result = get_longitudinal_limits(ay)

    expected_fraction = math.sqrt(
        1.0 - (ay / AY_MAX) ** 2
    )

    expected_acceleration = AX_ACCEL_MAX * expected_fraction
    expected_braking = -AX_BRAKE_MAX * expected_fraction

    assert result.feasible is True
    assert math.isclose(
        result.ax_accel_max,
        expected_acceleration,
    )
    assert math.isclose(
        result.ax_brake_max,
        expected_braking,
    )


def test_nine_metre_per_second_squared_lateral_acceleration():
    """
    Test a representative lateral acceleration value.

    This is useful because it is neither zero nor at the edge of the
    envelope.
    """

    ay = 9.0

    result = get_longitudinal_limits(ay)

    expected_fraction = math.sqrt(
        1.0 - (ay / AY_MAX) ** 2
    )

    assert result.feasible is True
    assert math.isclose(
        result.ax_accel_max,
        AX_ACCEL_MAX * expected_fraction,
    )
    assert math.isclose(
        result.ax_brake_max,
        -AX_BRAKE_MAX * expected_fraction,
    )


def test_maximum_lateral_acceleration():
    """
    At the maximum lateral acceleration, the entire G-G capability is being
    used laterally.

    Therefore there should be no longitudinal acceleration or braking
    capability remaining.
    """

    result = get_longitudinal_limits(AY_MAX)

    assert result.feasible is True
    assert math.isclose(result.ax_accel_max, 0.0)
    assert math.isclose(result.ax_brake_max, 0.0)


def test_negative_maximum_lateral_acceleration():
    """
    The current model is symmetric for left and right turns.

    Therefore -AY_MAX should produce the same result as +AY_MAX.
    """

    result = get_longitudinal_limits(-AY_MAX)

    assert result.feasible is True
    assert math.isclose(result.ax_accel_max, 0.0)
    assert math.isclose(result.ax_brake_max, 0.0)


def test_lateral_acceleration_above_limit():
    """
    Lateral acceleration above AY_MAX is physically infeasible according
    to the current vehicle model.

    The function should report this rather than silently clamping the
    requested value.
    """

    result = get_longitudinal_limits(AY_MAX + 0.1)

    assert result.feasible is False
    assert result.ax_accel_max == 0.0
    assert result.ax_brake_max == 0.0


def test_negative_lateral_acceleration_above_limit():
    """
    The infeasibility check should also work for excessive negative
    lateral acceleration.
    """

    result = get_longitudinal_limits(-(AY_MAX + 0.1))

    assert result.feasible is False
    assert result.ax_accel_max == 0.0
    assert result.ax_brake_max == 0.0



def test_positive_and_negative_lateral_acceleration_are_symmetric():
    """
    The current model does not distinguish between left and right turns.

    Therefore +ay and -ay should produce identical longitudinal limits.
    """

    ay = 8.0

    positive_result = get_longitudinal_limits(ay)
    negative_result = get_longitudinal_limits(-ay)

    assert positive_result.feasible == negative_result.feasible

    assert math.isclose(
        positive_result.ax_accel_max,
        negative_result.ax_accel_max,
    )

    assert math.isclose(
        positive_result.ax_brake_max,
        negative_result.ax_brake_max,
    )



def test_return_type():
    """
    Verify that the function returns the expected LongitudinalLimits
    dataclass.
    """

    result = get_longitudinal_limits(5.0)

    assert isinstance(result, LongitudinalLimits)



def test_acceleration_result_lies_on_g_g_envelope():
    """
    Verify that the returned maximum acceleration actually satisfies the
    acceleration ellipse.

    This is an important test because it checks the underlying physics
    rather than only checking a few expected numbers.
    """

    ay_values = [0.0, 2.0, 5.0, 8.0, 10.0, 11.5, AY_MAX]

    for ay in ay_values:
        result = get_longitudinal_limits(ay)

        assert result.feasible is True

        g_g_value = (
            (result.ax_accel_max / AX_ACCEL_MAX) ** 2
            + (ay / AY_MAX) ** 2
        )

        assert math.isclose(g_g_value, 1.0)


def test_braking_result_lies_on_g_g_envelope():
    """
    Verify that the returned maximum braking acceleration satisfies the
    braking ellipse.

    ax_brake_max is negative, but squaring it makes the sign irrelevant
    for the G-G equation.
    """

    ay_values = [0.0, 2.0, 5.0, 8.0, 10.0, 11.5, AY_MAX]

    for ay in ay_values:
        result = get_longitudinal_limits(ay)

        assert result.feasible is True

        g_g_value = (
            (result.ax_brake_max / AX_BRAKE_MAX) ** 2
            + (ay / AY_MAX) ** 2
        )

        assert math.isclose(g_g_value, 1.0)



def test_acceleration_is_positive():
    """
    Maximum forward acceleration should always be positive for a feasible
    state, except at the maximum lateral acceleration where it becomes zero.
    """

    for ay in [0.0, 4.0, 8.0, 11.0]:
        result = get_longitudinal_limits(ay)

        assert result.feasible is True
        assert result.ax_accel_max >= 0.0


def test_braking_is_negative():
    """
    Maximum braking acceleration should always be negative for a feasible
    state, except at the maximum lateral acceleration where it becomes zero.
    """

    for ay in [0.0, 4.0, 8.0, 11.0]:
        result = get_longitudinal_limits(ay)

        assert result.feasible is True
        assert result.ax_brake_max <= 0.0



def test_longitudinal_acceleration_decreases_with_lateral_acceleration():
    """
    As lateral acceleration increases, available longitudinal acceleration
    should never increase.

    This is a fundamental property of the G-G envelope.
    """

    previous = AX_ACCEL_MAX

    for ay in [1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 11.0, 12.0]:
        result = get_longitudinal_limits(ay)

        assert result.feasible is True
        assert result.ax_accel_max <= previous

        previous = result.ax_accel_max


def test_braking_capability_decreases_with_lateral_acceleration():
    """
    As lateral acceleration increases, the magnitude of available braking
    should decrease.

    Because braking values are negative, we compare their absolute values.
    """

    previous = AX_BRAKE_MAX

    for ay in [1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 11.0, 12.0]:
        result = get_longitudinal_limits(ay)

        assert result.feasible is True

        current = abs(result.ax_brake_max)

        assert current <= previous

        previous = current