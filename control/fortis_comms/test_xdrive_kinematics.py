from xdrive_kinematics import xdrive_fk_solver, xdrive_ik_solver, H, MAX_WHEEL_SPEED
import numpy as np

def main():
    tests = [
        ("Pure forward",  1, 0, 0),
        ("Pure strafe",   0, 1, 0),
        ("Pure rotation", 0, 0, 1),
        ("Diagonal",      1, 1, 0),
        ("Combined",      1, 0.5, 0.3),
    ]

    for label, vx, vy, omega in tests:
        ik = xdrive_ik_solver(vx, vy, omega)
        fk = xdrive_fk_solver(ik)

        expected = np.array([vx, vy, omega], dtype=float)
        raw_max = np.max(np.abs(H @ expected))
        scale = max(1.0, raw_max / MAX_WHEEL_SPEED)
        expected /= scale

        print(f"{label}: IK={ik}, FK={fk}, expected={expected}")
        assert np.allclose(fk, expected, atol=1e-6), f"FAIL: {label}"

    print("All tests passed.")

if __name__ == "__main__":
    main()