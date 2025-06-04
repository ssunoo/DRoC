
# Vehicle Routing Problem with Time Windows and Duration Limit (VRPTWL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, depot
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint and Duration Limit.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,              # slack_max: allow waiting time (as per example)
        int(duration_limit),  # capacity: maximum time per vehicle (duration limit)
        False,           # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot).
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )

    # Instantiate route start and end times to produce feasible times.
    # This also helps in minimizing the total time.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i))
        )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # Set a time limit for the solver to prevent it from running indefinitely.
    search_parameters.time_limit.seconds = 60  # Example: 60 seconds

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    obj = None # Initialize obj to None
    if solution:
        obj = solution.ObjectiveValue()

    return obj
