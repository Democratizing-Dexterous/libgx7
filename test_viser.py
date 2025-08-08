from __future__ import annotations

import time
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation as R
import tyro
from robot_descriptions.loaders.yourdfpy import load_robot_description

import viser
from viser.extras import ViserUrdf
import yourdfpy

from robot import GX7




def create_robot_control_sliders(
    server: viser.ViserServer, viser_urdf: ViserUrdf, robot
) -> tuple[list[viser.GuiInputHandle[float]], list[float]]:
    
    slider_handles: list[viser.GuiInputHandle[float]] = []
    initial_config: list[float] = []
    for joint_name, (
        lower,
        upper,
    ) in viser_urdf.get_actuated_joint_limits().items():
        lower = lower if lower is not None else -np.pi
        upper = upper if upper is not None else np.pi
        initial_pos = 0.0 
        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=1e-3,
            initial_value=initial_pos,
        )
        
        def update_joints(data):
            viser_urdf.update_cfg(
                np.array([slider.value for slider in slider_handles])
            )
            robot.setJPVs([slider.value for slider in slider_handles], [2]*len(slider_handles))
        
        slider.on_update(  # When sliders move, we update the URDF configuration.
                update_joints
            )
        
        slider_handles.append(slider)
        initial_config.append(initial_pos)
    return slider_handles, initial_config


def main(
    robot_type: Literal[
        "panda",
        "ur10",
        "cassie",
        "allegro_hand",
        "barrett_hand",
        "robotiq_2f85",
        "atlas_drc",
        "g1",
        "h1",
        "anymal_c",
        "go2",
    ] = "panda",
    load_meshes: bool = True,
    load_collision_meshes: bool = False,
) -> None:
    
    # Start viser server.
    server = viser.ViserServer()
    robot_base = server.scene.add_frame("/robot", show_axes=False)
    # Calculate quaternion for -90 degrees rotation around y-axis using scipy
    rotation = R.from_euler('y', -90, degrees=True)
    # Convert to quaternion in wxyz format (scipy uses xyzw by default)
    quat_xyzw = rotation.as_quat()  # returns x, y, z, w
    quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # convert to w, x, y, z
    robot_base.wxyz = quat_wxyz  # quaternion in wxyz format

    # Load URDF.
    #
    # This takes either a yourdfpy.URDF object or a path to a .urdf file.
    # urdf = load_robot_description(
    #     "urdf/GX7_250721.urdf",
    #     load_meshes=load_meshes,
    #     build_scene_graph=load_meshes,
    #     load_collision_meshes=load_collision_meshes,
    #     build_collision_scene_graph=load_collision_meshes,
    # )
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=yourdfpy.URDF.load('urdf/GX7_250721.urdf'),
        load_meshes=load_meshes,
        load_collision_meshes=load_collision_meshes,
        collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
        root_node_name="/robot"
    )

    robot = GX7(100, control_mode='pv')
    # Create sliders in GUI that help us move the robot joints.
    with server.gui.add_folder("Joint position control"):
        (slider_handles, initial_config) = create_robot_control_sliders(
            server, viser_urdf, robot
        )

    # Add visibility checkboxes.
    with server.gui.add_folder("Visibility"):
        show_meshes_cb = server.gui.add_checkbox(
            "Show meshes",
            viser_urdf.show_visual,
        )
        show_collision_meshes_cb = server.gui.add_checkbox(
            "Show collision meshes", viser_urdf.show_collision
        )

    @show_meshes_cb.on_update
    def _(_):
        viser_urdf.show_visual = show_meshes_cb.value

    @show_collision_meshes_cb.on_update
    def _(_):
        viser_urdf.show_collision = show_collision_meshes_cb.value

    # Hide checkboxes if meshes are not loaded.
    show_meshes_cb.visible = load_meshes
    show_collision_meshes_cb.visible = load_collision_meshes

    # Set initial robot configuration.
    viser_urdf.update_cfg(np.array([0]*7))

    # Create grid.
    trimesh_scene = viser_urdf._urdf.scene or viser_urdf._urdf.collision_scene
    server.scene.add_grid(
        "/grid",
        width=2,
        height=2,
        position=(
            0.0,
            0.0,
            # Get the minimum z value of the trimesh scene.
            trimesh_scene.bounds[0, 2] if trimesh_scene is not None else 0.0,
        ),
    )

    # Create joint reset button.
    reset_button = server.gui.add_button("Reset")

    @reset_button.on_click
    def _(_):
        for s, init_q in zip(slider_handles, initial_config):
            s.value = init_q

    robot.run()
    # Sleep forever.
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    main()