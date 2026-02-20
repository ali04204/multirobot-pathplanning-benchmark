# TEST WITH SPECIFIC PLANNERS (demos)
### Run prioritized planner on 2D hallway
python3 examples/run_planner.py rai.2d_hallway --planner=prioritized --max_time=30 --optimize

### Run composite PRM on box stacking
python3 examples/run_planner.py rai.box_stacking --planner=composite_prm --max_time=30 --optimize

### Run RRT* on mobile wall navigation
python3 examples/run_planner.py rai.mobile_wall --planner=rrt_star --max_time=30 --optimize


# RUN AN EXPERIMENT
python3 examples/run_experiment.py configs/demo/mobile_wall.json


# PYTESTS
pytest tests/test_planners.py -v
pytest tests/test_envs.py -v


# VISUALIZE AVAILABLE ENVIRONMENTS
python3 examples/show_problems.py --mode list_all
python3 examples/show_problems.py rai.box_stacking --mode modes