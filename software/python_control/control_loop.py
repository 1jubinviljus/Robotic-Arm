import time
from .fingertip_tracking import FingertipTracker
from simulation.pybullet_env.arm_simulation import ArmSimulation

def main():
    print("Starting tracker and simulation...")
    tracker = FingertipTracker()
    sim = ArmSimulation()

    tracker.start()
    try:
        while True:
            fingertip_pos = tracker.get_fingertip_position()
            print("Fingertip position:", fingertip_pos)
            if fingertip_pos is None:
                time.sleep(0.01)
                continue

            x, y = fingertip_pos
            print(f"Moving target to: x={x:.3f}, y={y:.3f}")
            sim.move_to_target(x, 0, y)
            sim.step_simulation()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, stopping...")
    except Exception as e:
        print("Exception:", e)
    finally:
        tracker.stop()
        sim.disconnect()
        print("Cleaned up and exiting.")

if __name__ == "__main__":
    main()
