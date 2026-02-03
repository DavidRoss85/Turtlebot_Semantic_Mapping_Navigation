from custom_navigator.mapping.inflated_costmap import InflatedCostmap

import numpy as np

def test_get_inflated_costmap():

    input_grid = np.zeros((10,10), dtype=np.int16)
    costmap = InflatedCostmap(
        occupancy_grid=input_grid,
        inflation_radius=1.0,
        resolution=1.0,
        threshold=50
    )

    output_grid = costmap.get_inflated_costmap()
    assert np.array_equal(input_grid,output_grid)