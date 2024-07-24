import os
import shutil
import yaml

from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner

CONFIG_FILE = "panda_config.yaml"
with open(CONFIG_FILE, 'r') as config_file:
    config = yaml.safe_load(config_file)
    CONFIG_PROBLEM = config['problem']
    CONFIG_FABRICS = config['fabrics']


def set_planner(degrees_of_freedom: int = 7):
    """
    Initializes the fabric planner for the panda robot.

    Reparameterization is done in the panda_config.yaml file.

    Params
    ----------
    degrees_of_freedom: int
        Degrees of freedom of the robot (default = 7)
    """
    absolute_path = os.path.dirname(os.path.abspath(__file__))
    with open(absolute_path + "/panda_for_fk.urdf", "r", encoding="utf-8") as file:
        urdf = file.read()
    forward_kinematics = GenericURDFFk(
        urdf,
        root_link="panda_link0",
        end_links=["panda_link9"],
    )
    planner = ParameterizedFabricPlanner(
        degrees_of_freedom,
        forward_kinematics,
    )
    planner.load_fabrics_configuration(CONFIG_FABRICS)
    planner.load_problem_configuration(CONFIG_PROBLEM)
    planner.concretize()
    planner.serialize('controller.pbz2')
    planner.export_as_c('controller.c')

if __name__ == "__main__":
    set_planner()
