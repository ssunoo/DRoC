# Prize Collecting Vehicle Routing Problem with Time Windows and Multiple Depots (PCVRPTWMD)
# [('user', 'The solution failed the code execution test: Error: \'bool\' object has no attribute \'CumulVar\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpqt9kdle1.py", line 86, in <module>\n    result = solve(time_matrix, time_windows, starts, ends, num_vehicle, prizes, max_time)\n  File "/tmp/tmpqt9kdle1.py", line 36, in solve\n    time_dimension.CumulVar(index).SetRange(window[0], window[1])\nAttributeError: \'bool\' object has no attribute \'CumulVar\'\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicle: int, prizes: list, max_time: int):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, starts, ends)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for travel times.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension.
    time_dimension = routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_time,  # maximum travel time
        False,  # Don't force start cumul to zero.
        'Time')

    # Check if the dimension was added successfully.
    if time_dimension is None:
        raise RuntimeError("Failed to add Time dimension")

    # Set time window constraints for each location.
    for location_idx, window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(window[0], window[1])

    # Set time window constraints for each vehicle start node.
    for start in starts:
        index = manager.NodeToIndex(start)
        time_dimension.CumulVar(index).SetRange(time_windows[start][0], time_windows[start][1])

    # Add prize collection as a disjunction with a penalty.
    for node_idx, prize in enumerate(prizes):
        if prize > 0:
            index = manager.NodeToIndex(node_idx)
            routing.AddDisjunction([index], prize)

    # Add constraint for maximum travel time.
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index))
        end_index = routing.End(vehicle_id)
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(end_index))

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1