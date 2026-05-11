"""
urdf_cleanup -- host-level dev tool to post-process OnShape URDF exports.

OnShape's URDF export plug-in produces a tree that ROS 2 Humble cannot
consume as-is: mangled link names with double underscores, dozens of
nuisance fixed joints, wheel hubs hanging off a synthetic ``root`` node,
and a ROS 1 ``launch.xml`` instead of ``launch.py``. This package
transforms that raw export into a clean URDF plus the launch + RViz
files the FORTIS stack actually loads.

The tool lives outside ``src/`` because it runs at the host level on
checked-out URDF artefacts; it is not part of any colcon package and
is not built by ``colcon build``. Tests live in ``../tests/`` and are
run with ``pytest`` directly.

Pipeline (see ``main.run``):

    raw URDF + meshes/
        |
        v
    parser (lxml -> document)
        |
        v
    link_filter (drop nuisance fixed-joint trees)
        |
        v
    topology_fixer (re-parent wheel hubs from synthetic ``root``
                    onto the chassis frame)
        |
        v
    renamer (mangled-name -> clean-name map; default config for FORTIS
             chassis, overrideable via --mapping)
        |
        v
    ros2_control (inject <ros2_control> + hardware_interface blocks for
                  the wheel revolute joints)
        |
        v
    launch_generator + rviz_generator + mesh_renamer
        |
        v
    cleaned URDF + launch.py + rviz config + renamed meshes/
"""

__version__ = "0.1.0"
