
# Travelling Salesman Problem with Time Windows and Service Time (TSPTWS)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    # Create the routing index manager.
    num_locations = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(
        num_locations, 1, depot  # num_locations: number of locations, 1: number of vehicles, depot: index of the depot
    )

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time_horizon = max(max(time_window) for time_window in time_windows) + max(service_time)
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        routing.AddAtTimeWindow(index, service_time[location_idx], time_window[0], time_window[1])
    routing.AddAtTimeWindow(manager.NodeToIndex(depot), service_time[depot], time_windows[depot][0], time_windows[depot][1])
    dimension_name = "Time"
    routing.AddDimension(
        transit_callback_index,
        time_horizon,
        time_horizon,  # Allow the vehicle to wait at the start.
        False,  # Don't force start cumul to zero.
        dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(dimension_name)
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        obj = solution.ObjectiveValue()
    return obj
