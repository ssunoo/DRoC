
# Prize Collecting Vehicle Routing Problem with Time Windows and Multiple Depots (PCVRPTWMD)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, starts, ends)

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc.
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
    time_window = "Time"
    routing.AddDimension(
        transit_callback_index,  # transit callback index
        max_time,  # allow waiting time
        max_time,  # vehicle maximum travel time
        False,  # start cumul to zero
        time_window)
    time_dimension = routing.GetDimensionOrDie(time_window)
    # Add time window for each node
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == starts[0] or location_idx == ends[0]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add prize for each node.
    for i in range(len(prizes)):
        if i != starts[0] and i != ends[0]:
            routing.SetNodeCost(manager.NodeToIndex(i), -prizes[i])

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        obj = solution.ObjectiveValue()

    return obj
