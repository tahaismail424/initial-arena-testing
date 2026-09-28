# 7. Hands-on study plan

Each exercise teaches one layer of the stack and ends with something to write down. Keep notes in `notes/` in this repo: what you changed, what you saw, and a screenshot if useful. Do them roughly in order; each builds on the last.

## Week 1: see the plumbing

**E1. Trace a velocity command.** Launch the hospital. In a second sourced terminal:
```bash
ros2 topic info -v /task_generator_node/jackal/cmd_vel
ros2 topic info -v /task_generator_node/jackal/cmd_vel_nav
ros2 topic hz /task_generator_node/jackal/cmd_vel_nav
```
Draw the chain from controller to Gazebo yourself. *Question:* why is `cmd_vel_nav` published at about 1 Hz?

**E2. RViz vs Gazebo.** Hide and show RViz displays one at a time: map, global costmap, local costmap, lidar, plan, local_plan, pedestrians. For each, write down which node publishes it (`ros2 topic info -v`).

**E3. TF tree.** `ros2 run tf2_tools view_frames` and open `frames.pdf`. Identify `map → jackal/odom → jackal/base_link → jackal/lidar_link`. Who publishes each edge?

**E4. The merged Nav2 config.** `ros2 param dump /task_generator_node/jackal/controller_server`. Find each value's source among the five YAML files in the merge chain ([04 E](04-configuring-sim-and-robot.md#e-the-nav2-yaml-merge-chain)).

**E5. Read the task generator loop.** Read `task_generator/node.py` (`_check_task_status`, `reset_task`) and `robot_manager.py` (`_launch_robot`, `_goal_status_callback`). Then call `ros2 service call /task_generator_node/reset_task std_srvs/srv/Empty` and watch the logs.

## Week 2: change things and observe

**E6. Fix the 1 Hz controller** (your first real config change). In `simulation-setup/configs/nav2/nav2.yaml`, set `controller_frequency: 10.0`. Relaunch without rebuilding, since it's an existing file. Does DWB still spin at the hospital start? Record time-to-goal before and after.

**E7. Live parameter tuning.** While running: `ros2 param set /task_generator_node/jackal/controller_server FollowPath.max_vel_x 0.5`. Watch the behaviour change. Then make it permanent in `controllers/dwb/controller_config.yaml`.

**E8. Swap controllers.** Run the same scenario with `local_planner:=mppi`, `regulated_pure_pursuit`, `rotation_shim`. Notice RPP just follows the path, with no dynamic-obstacle reasoning. Why does that matter in a crowd?

**E9. Fix the footprint** in `entities/robots/jackal/model_params.yaml`. Watch the footprint polygon change in RViz.

**E10. Make your own scenario.** Copy `hospital/scenarios/default.json` to `my_small.json`, keep 5 pedestrians, and give the robot a shorter route. Rebuild `arena_simulation_setup`, since it's a new file. Set `task.scenario.file: my_small.json` in `task_generator.yaml`. Relaunch.

**E11. Pedestrian behaviour.** In a copy of `scenarios/hallway_tame.yaml`, set one class's `social_force_factor` to 0 and watch them walk through each other (`check_crowd.py` shows the min gap collapsing). Then try behaviour types `SCARED` (4) / `CURIOUS` (5) for the class walking toward the robot.

**E12. Lidar.** Make the Jackal lidar a true 2D scan: vertical `samples 1`, 270°, 720 samples, 30 m, so it matches `model_params.yaml`. Check with `ros2 topic echo --once .../lidar --field angle_min`.

## Week 3: evaluation and learned policies

**E13. Record and score a run.** Set `timeout: 120.0` and `episodes: 5` in `task_generator.yaml`. Launch with `record_data_dir:=dwb_baseline headless:=1`, then run `arena_evaluation`'s `get_metrics.py` on the output. Read `evaluation/arena_evaluation/README.md` first, and locate where the data folder lands.

**E14. Install nav2py plus one learned planner** (DRL-VO is the best documented), following [05](05-policies-and-pytorch.md#installing-the-learned-planners-drl-vo-crowdnav-sicnav). Read its Python `__main__.py` and write down its exact observation vector and action range.

**E15. First head-to-head.** DWB vs MPPI vs DRL-VO on the same scenario, same limits, 10 episodes each. Plot success/collision/timeout rates and time-to-goal. This is the table to bring to Dr. Lyu.

## Week 4: your own model

**E16. Toy PyTorch policy, rclpy route.** A tiny node that reads lidar and goal and outputs `(v, ω)` from a hand-made `nn.Module`, even a random or linear one. The goal is plumbing, not performance. Remember to stop Nav2's controller from fighting you ([05](05-policies-and-pytorch.md#alternative-for-quick-prototyping-a-plain-rclpy-node-no-nav2-plugin)).

**E17. Same model as a nav2py plugin.** Port E16 into a nav2py package, then run it with `local_planner:=my_policy`. Now it's benchmarkable alongside everything else.

## Questions to be able to answer by the end

- What's the difference between a world, a scenario, and a task mode?
- Which file would you edit to change (a) max speed, (b) robot size, (c) lidar range, (d) pedestrian count, (e) physics step, (f) episode timeout?
- Which edits need a `colcon build` and which don't, and why?
- How does a Python PyTorch model end up moving a wheel in Gazebo?
- What information does each compared policy get, and is the comparison fair?
