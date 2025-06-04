
# Vehicle Routing Problem with Time Windows and Open Start and End locations (OVRPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, 0, 0) # Initialize the routing index manager with the number of locations, number of vehicles, depot start index, and depot end index.

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager) # Create the routing model using the index manager.

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index) # Convert the routing variable index to the node index.
        to_node = manager.IndexToNode(to_index) # Convert the routing variable index to the node index.
        return time_matrix[from_node][to_node] # Return the travel time between the two nodes.

    transit_callback_index = routing.RegisterTransitCallback(time_callback) # Register the transit callback and get its index.

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index) # Set the cost of each arc using the transit callback.

    # Add Time Windows constraint.
    time_window = "TimeWindow" # Define the name of the time window dimension.
    routing.AddDimension(
        transit_callback_index,  # transit callback index
        30,  # allow waiting time
        30,  # vehicle maximum travel time
        False,  # start cumul to zero
        time_window) # Add a time window dimension to the model.
    time_window_dimension = routing.GetDimensionOrDie(time_window) # Get the time window dimension.
    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx) # Get the index of the location.
        time_window_dimension.SetCumulVarSoftLowerBound(index, time_window[0]) # Set the soft lower bound for the time window.
        time_window_dimension.SetCumulVarSoftUpperBound(index, time_window[1]) # Set the soft upper bound for the time window.
    # Add time window constraints for depot
    index = manager.NodeToIndex(0) # Get the index of the depot.
    time_window_dimension.SetCumulVarSoftLowerBound(index, time_windows[0][0]) # Set the soft lower bound for the time window.
    time_window_dimension.SetCumulVarSoftUpperBound(index, time_windows[0][1]) # Set the soft upper bound for the time window.

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters() # Create the search parameters.
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC) # Set the first solution strategy.

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters) # Solve the routing problem.

    # Print solution on console.
    if solution:
        obj = solution.ObjectiveValue() # Get the objective value.
    return obj
