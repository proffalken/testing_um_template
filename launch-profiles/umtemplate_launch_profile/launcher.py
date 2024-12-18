"""Launch nodes for this launch profile."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from pydantic import BaseModel

from node_helpers import launching
from node_helpers.parameters import ParameterLoader

# Import the forklift URDF module so it register itself with the URDFConstants
from um_test import example_urdf

class MetaParameters(BaseModel):
    """This is a great place to put parameters that affect the generation of the launch
    file. Don't put node specific configuration in here, rather, put configuration for
    what nodes you want to be created in the first place.

    Read more about this functionality under docs/parameters.rst
    """

    urdf_modules_to_load: list[launching.URDFModuleNodeFactory.Parameters]
    """This is an example of dynamically loading an arbitrary number of URDFs.

    This is set in the `/robot/launch-profile/parameters.yaml` under `meta_parameters`.
    """


def generate_launch_description() -> LaunchDescription:
    # Create a parameter loader to parse all yaml files in the launch-profile/parameters
    # directory, and then apply overrides from the override file, if one exists.
    param_loader: ParameterLoader[MetaParameters] = ParameterLoader(
        parameters_directory=Path("/robot/launch-profile/parameters/"),
        override_file=Path("/robot/launch-profile/parameters.override.yaml"),
        meta_parameters_schema=MetaParameters,
    )

    rviz_config = launching.required_file("/robot/launch-profile/rviz-config.rviz")

    urdf_node_factories = (
        launching.URDFModuleNodeFactory(
            parameters=node_factory_params
        )
        for node_factory_params in param_loader.meta_parameters.urdf_modules_to_load
    )
    urdf_nodes = []
    for urdf_node_factory in urdf_node_factories:
        urdf_nodes += urdf_node_factory.create_nodes()

    launch_description = [
        *urdf_nodes,
        Node(
            package="um_test",
            executable="run_UmTemplate",
            name="UmTemplate",
            parameters=[param_loader.ros_parameters_file],
            namespace="example_node_namespace",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", [str(rviz_config)]],
        ),
        Node(
            namespace="urdf_arrangement",
            package="node_helpers",
            executable="interactive_transform_publisher",
            parameters=[param_loader.ros_parameters_file],
        ),
    ]
    return LaunchDescription(launch_description)
