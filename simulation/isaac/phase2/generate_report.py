"""
Phase 2 Step 6: Generate Parameter Report

Reads all test data from Steps 3-5 and compiles engineering parameters.

Run:
  python E:\FORTIS\Optimizer\phase2\generate_report.py
  (or use Isaac Sim python.bat if needed, but this script has no sim dependencies)
"""
import os, sys, json, math

PHASE2_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(PHASE2_DIR, "data")
REPORT_PATH = os.path.join(PHASE2_DIR, "phase2_results.txt")

WHEEL_RADIUS_IN = 3.0
WHEEL_RADIUS_M = WHEEL_RADIUS_IN * 0.0254


def load_json(filename):
    path = os.path.join(DATA_DIR, filename)
    if not os.path.exists(path):
        print(f"WARNING: {path} not found, skipping.")
        return None
    with open(path) as f:
        return json.load(f)


def generate():
    lines = []
    def p(s=""):
        lines.append(s)
        print(s)

    p("=" * 70)
    p("FORTIS Phase 2: Engineering Parameter Report")
    p("=" * 70)

    # -- Rotation results --
    rot_data = load_json("rotation_results.json")
    if rot_data:
        results = rot_data.get("results", [])
        p("\n## 1. Rotation Test Results")
        p(f"   Configurations tested: {len(results)}")

        successful = [r for r in results if r.get("full_360") and not r.get("collision") and not r.get("error")]
        failed = [r for r in results if not r.get("full_360") and not r.get("error")]
        collisions = [r for r in results if r.get("collision")]

        p(f"   Successful 360-deg rotations: {len(successful)}")
        p(f"   Failed (incomplete): {len(failed)}")
        p(f"   Collisions: {len(collisions)}")

        if successful:
            # Minimum torque
            min_t = min(successful, key=lambda r: r["max_torque_nm"])
            p(f"\n   Minimum torque for 360-deg rotation:")
            p(f"     {min_t['max_torque_nm']} Nm at {min_t['vel_ms']} m/s "
              f"(completed in {min_t['time_to_360_s']}s)")

            # Maximum safe speed
            max_v = max(successful, key=lambda r: r["vel_ms"])
            p(f"\n   Maximum safe rotation speed:")
            p(f"     {max_v['vel_ms']} m/s wheel speed "
              f"({max_v.get('peak_yaw_rate_dps', '?')} deg/s peak yaw rate)")

            # Fastest rotation
            fastest = min(successful, key=lambda r: r["time_to_360_s"] if r["time_to_360_s"] else 999)
            p(f"\n   Fastest safe 360-deg rotation:")
            p(f"     {fastest['time_to_360_s']}s at v={fastest['vel_ms']} m/s, "
              f"T={fastest['max_torque_nm']} Nm")

            # Summary table
            p(f"\n   {'Vel (m/s)':>10} {'Torque (Nm)':>12} {'Time (s)':>9} {'Rate (d/s)':>10}")
            p(f"   {'-'*45}")
            for r in sorted(successful, key=lambda r: (r["vel_ms"], r["max_torque_nm"])):
                p(f"   {r['vel_ms']:>10.3f} {r['max_torque_nm']:>12} "
                  f"{r['time_to_360_s']:>9} {r.get('avg_yaw_rate_dps', '?'):>10}")

    # -- Driving results --
    drv_data = load_json("driving_results.json")
    if drv_data:
        tests = drv_data.get("tests", {})
        p("\n\n## 2. Driving Test Results")
        p(f"   Max torque used: {drv_data.get('max_torque', '?')} Nm")

        for test_name in ["straight", "arc_gentle", "arc_tight"]:
            test_results = tests.get(test_name, [])
            if not test_results:
                continue

            p(f"\n   --- {test_name} ---")
            safe = [r for r in test_results if not r.get("collision") and not r.get("error")]
            coll = [r for r in test_results if r.get("collision")]

            p(f"   Safe runs: {len(safe)}, Collisions: {len(coll)}")

            if safe:
                max_safe = max(safe, key=lambda r: r["vel_ms"])
                p(f"   Max safe speed: {max_safe['vel_ms']} m/s")
                p(f"   At max speed -- orbit: {max_safe['orbit_deg']}deg, "
                  f"drift: {max_safe['lateral_drift_in']}\"")

                p(f"\n   {'Vel (m/s)':>10} {'Orbit (deg)':>12} {'Drift (in)':>11} {'Avg R (in)':>11}")
                p(f"   {'-'*48}")
                for r in safe:
                    p(f"   {r['vel_ms']:>10.2f} {r['orbit_deg']:>12.1f} "
                      f"{r['lateral_drift_in']:>11.1f} {r['avg_radius_in']:>11.1f}")

    # -- Arm stability results --
    arm_data = load_json("arm_stability_results.json")
    if arm_data:
        results = arm_data.get("results", [])
        p("\n\n## 3. Arm Stability Results")

        stable_count = sum(1 for r in results if r.get("stable"))
        p(f"   Poses tested: {len(results)}, Stable: {stable_count}")

        p(f"\n   {'Pose':>18} {'Stable':>7} {'dRoll':>7} {'dPitch':>8} {'dZ (in)':>8}")
        p(f"   {'-'*52}")
        for r in results:
            if "error" in r:
                p(f"   {r['pose']:>18} {'ERROR':>7}")
                continue
            p(f"   {r['pose']:>18} {'YES' if r['stable'] else 'NO':>7} "
              f"{r['roll_change_deg']:>7.1f} {r['pitch_change_deg']:>8.1f} "
              f"{r['z_change_in']:>8.2f}")

        unstable = [r for r in results if not r.get("stable") and not r.get("error")]
        if unstable:
            worst = max(unstable, key=lambda r: r.get("roll_change_deg", 0) + r.get("pitch_change_deg", 0))
            p(f"\n   Worst-case pose for tipping: {worst['pose']}")
        else:
            p(f"\n   All poses are stable.")

    # -- Derived engineering parameters --
    p("\n\n## 4. Derived Engineering Parameters")

    p("\n   ### Chassis/Drive:")
    if rot_data:
        successful = [r for r in rot_data.get("results", [])
                      if r.get("full_360") and not r.get("collision") and not r.get("error")]
        if successful:
            min_t = min(successful, key=lambda r: r["max_torque_nm"])
            p(f"   Min wheel torque for 360-deg rotation: {min_t['max_torque_nm']} Nm")
            max_v = max(successful, key=lambda r: r.get("peak_yaw_rate_dps", 0))
            p(f"   Max safe rotation rate: {max_v.get('peak_yaw_rate_dps', '?')} deg/s")

    if drv_data:
        straight = drv_data.get("tests", {}).get("straight", [])
        safe_straight = [r for r in straight if not r.get("collision") and not r.get("error")]
        if safe_straight:
            max_fwd = max(safe_straight, key=lambda r: r["vel_ms"])
            p(f"   Max safe forward speed: {max_fwd['vel_ms']} m/s "
              f"({max_fwd['vel_ms'] * 3.281:.1f} ft/s)")

    p(f"\n   Graphite friction baseline: mu = 0.3")
    p(f"   Wheel material friction set: mu = 0.5 (rubber on graphite)")

    # Motor sizing
    p("\n   ### Motor/Gearbox Recommendation:")
    if rot_data:
        successful = [r for r in rot_data.get("results", [])
                      if r.get("full_360") and not r.get("collision") and not r.get("error")]
        if successful:
            min_t = min(successful, key=lambda r: r["max_torque_nm"])
            reqd_torque = min_t["max_torque_nm"]
            safety_factor = 1.5
            design_torque = reqd_torque * safety_factor

            p(f"   Required wheel torque: {reqd_torque} Nm (min for rotation)")
            p(f"   Design torque (1.5x safety): {design_torque:.1f} Nm")

            # Motor sizing example
            # target: 0.2 m/s max speed with 3" tire
            max_wheel_rpm = (0.2 / WHEEL_RADIUS_M) * 60 / (2 * math.pi)
            p(f"   Target max wheel speed: {max_wheel_rpm:.0f} RPM (at 0.2 m/s)")

            # Typical small DC motor: 6000 RPM, 0.05-0.1 Nm
            motor_rpm = 6000
            motor_torque = 0.08  # Nm
            gear_ratio = math.ceil(design_torque / motor_torque)
            output_rpm = motor_rpm / gear_ratio
            output_torque = motor_torque * gear_ratio * 0.85  # 85% efficiency
            output_speed = output_rpm * 2 * math.pi * WHEEL_RADIUS_M / 60

            p(f"\n   Suggested: 12V brushed DC motor ({motor_torque} Nm @ {motor_rpm} RPM)")
            p(f"   Gear ratio: {gear_ratio}:1 planetary")
            p(f"   Output: {output_torque:.1f} Nm @ {output_rpm:.0f} RPM")
            p(f"   Wheel speed: {output_speed:.2f} m/s ({output_speed*3.281:.1f} ft/s)")

            if output_torque >= design_torque:
                p(f"   Status: MEETS requirements ({output_torque:.1f} >= {design_torque:.1f} Nm)")
            else:
                p(f"   Status: INSUFFICIENT -- increase gear ratio or motor size")

    p("\n" + "=" * 70)
    p("End of Phase 2 Report")
    p("=" * 70)

    # Save report
    with open(REPORT_PATH, "w") as f:
        f.write("\n".join(lines))
    print(f"\nReport saved: {REPORT_PATH}")


if __name__ == "__main__":
    generate()
