from .vehicle_model import get_longitudinal_limits


def main():
    test_values = [
        0.0,
        3.0,
        6.0,
        9.0,
        12.0,
        13.0,
        -6.0,
        -12.0,
        -13.0,
    ]

    for ay in test_values:
        result = get_longitudinal_limits(ay)

        print(
            f"ay = {ay:6.2f} m/s² | "
            f"ax accel = {result.ax_accel_max:6.2f} m/s² | "
            f"ax brake = {result.ax_brake_max:6.2f} m/s² | "
            f"feasible = {result.feasible}"
        )


if __name__ == "__main__":
    main()