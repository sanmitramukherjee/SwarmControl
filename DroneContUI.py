"""
SwarmControl — Multi-Drone Ground Control Station
Usage:
  SITL:        python DroneContUI.py --sitl --drones 5762 5772 5782
  Real:        python DroneContUI.py --real --drones COM3:1 COM3:2
  + GUI:       python DroneContUI.py --sitl --drones 5762 5772 --control
  No browser:  python DroneContUI.py --sitl --drones 5762 --no-browser
"""

from swarmcontrol import swarm as _swarm_state
from swarmcontrol.cli import main


def __getattr__(name):
    return getattr(_swarm_state, name)


if __name__ == "__main__":
    main()
