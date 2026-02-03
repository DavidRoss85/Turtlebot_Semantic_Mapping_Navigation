from custom_navigator.planners.astar import AStarPlanner

import numpy as np

def test_plan():

    planner = AStarPlanner()
    test_grid = np.zeros((100,100),dtype=np.int16)
    start = (0,0)
    goal = (10,10)

    planner.set_grid(test_grid)
    
    plan = planner.plan(start=start,goal=goal)

    print(plan)

    assert plan is not None
    assert len(plan) > 100

