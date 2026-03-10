# Planner Commands

## Single Planner Runs

```bash
# Informed RRT* (50 sec)
# python3 examples/run_planner.py rai.2d_handover --planner=informed_rrt_star --max_time=50 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max

python3 examples/run_planner.py rai.2d_handover --planner=informed_rrt_star --max_time=30 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max --irrt.with_mode_validation=False

python3 examples/run_planner.py rai.piano --planner=informed_rrt_star --max_time=30 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max --irrt.with_mode_validation=False

python3 examples/run_planner.py rai.rfl_upright --planner=informed_rrt_star --max_time=30 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max --irrt.with_mode_validation=False

# Standard RRT* with informed sampling (50 sec) - default behavior
python3 examples/run_planner.py rai.2d_handover --planner=rrt_star --max_time=50 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max

# Standard RRT* WITHOUT informed sampling (50 sec)
# python3 examples/run_planner.py rai.2d_handover --planner=rrt_star --max_time=50 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max --rrt.informed_sampling=False

python3 examples/run_planner.py rai.2d_handover --planner=rrt_star --optimize --max_time=30 --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max --rrt.with_mode_validation=False --rrt.informed_sampling=False

# Bidirectional RRT* (50 sec)
python3 examples/run_planner.py rai.2d_handover --planner=birrt_star --max_time=50 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max

# PRM (50 sec)
python3 examples/run_planner.py rai.2d_handover --planner=composite_prm --max_time=50 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max

# AIT* (50 sec)
python3 examples/run_planner.py rai.2d_handover --planner=aitstar --max_time=50 --optimize --distance_metric=euclidean --per_agent_cost_function=euclidean --cost_reduction=max
```

## Informed RRT* with Custom Options

```bash
# With tree pruning disabled
python3 examples/run_planner.py rai.2d_handover --planner=informed_rrt_star --max_time=50 --optimize --irrt.prune_tree=False

# With rejection sampling instead of direct PHS
python3 examples/run_planner.py rai.2d_handover --planner=informed_rrt_star --max_time=50 --optimize --irrt.rejection_sampling=True

# With higher goal bias
python3 examples/run_planner.py rai.2d_handover --planner=informed_rrt_star --max_time=50 --optimize --irrt.p_goal=0.5
```

## Different Environments

```bash
# Mobile robots (simpler 2D)
python3 examples/run_planner.py mobile_four --planner=informed_rrt_star --max_time=30 --optimize

# Box sorting
python3 examples/run_planner.py rai.box_sorting --planner=informed_rrt_star --max_time=60 --optimize

# Piano mover
python3 examples/run_planner.py rai.piano_mover --planner=informed_rrt_star --max_time=60 --optimize
```

## Batch Comparison (Config File)

```bash
python3 examples/run_experiment.py configs/demo/mobile_wall_dep.json
```
