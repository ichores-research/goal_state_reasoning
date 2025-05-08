# Ichores Pipeline with LLM reasoner and Motion_stack

Check if you are on right git branch for LLM reasoner
```
cd ichores_pipeline_KRK_07052025/src/goal_state_reasoning/
git branch #should be prague_exepriment
```

## Start Motion_stack
Run melodic_noetic_bridge in docker
```
cd tiago_docker_pipeline
docker compose up -d

```

To check if it is running try dummy code from Motion_stack:
```
cd Motion_stack/bridge_noetic_melodic
python dummy_noetic.py
```

## Prepare LLM reasoner
To use LLM reasoner you need to have an Open AI API key saved in `ichores_pipeline_KRK_07052025/src/goal_state_reasoning/src/.env` file 
```
#.env
OPENAI_API_KEY=<your_key>
```
## Run ichores pipeline

All docker images are build in this PC. Pointing Gesture Detection module has been commented out of docker-compose and `test_obj_det_dev.py` as it is not needed for this task.

However, if you want to build images from scratch:
```
cd ichores_pipeline_KRK_07052025/compose/pipeline
docker compose build
```

### Start pipeline
```
cd ichores_pipeline_KRK_07052025/compose/pipeline
set -a && DATASET=ycb_ichores CONFIG=params_tiago_prague.yaml ROS_MASTER_URI=http://tiago-114c:11311 ROS_IP=10.68.0.128  docker compose
```

## Run LLM Agent \[EXPERIMENTAL\]

This module is best to be tried out in simulation first.
Before running on real robot, move the arms into a high position and set the robot close to sturophome table.
Set needed ycb objects on the table.
Check in rvix if the robot's camera can see the objects.


To call LLM reasoner service:
```
cd ichores_pipeline_KRK_07052025/src/goal_state_reasoning/docker
docker-compose exec goal_state_reasoning bash

#inside container
source catkin_ws/devel/setup.bash
rosservice call /LLM_plan_and_exec "command: '<your command>'
```

To see LLM outputs check container logs
```
# outside container in ichores_pipeline_KRK_07052025/src/goal_state_reasoning/docker
docker compose logs -f goal_state_reasoning 
```

