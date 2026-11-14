"""Load a scene file into the running CoppeliaSim (simulation must be stopped): python -m evaluation.load_scene <file.ttt>"""
import sys
import time

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    sys.path.append(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python')
    from zmqRemoteApi import RemoteAPIClient

sim = RemoteAPIClient().require("sim")
print("current scene:", sim.getStringParam(sim.stringparam_scene_path_and_name))
if sim.getSimulationState() != sim.simulation_stopped:
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.2)
sim.loadScene(sys.argv[1])
print("loaded scene:", sim.getStringParam(sim.stringparam_scene_path_and_name))
