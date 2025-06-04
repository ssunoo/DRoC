
# Vehicle Routing Problem with Time Windows and Multiple Depots and Duration Limit (VRPTWMDL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    obj = -1

    # Create the routing index manager.
    # The manager handles the mapping between node indices and routing indices.
    # It takes the number of nodes, number of vehicles, and lists of start/end depots.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicles, starts, ends
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    # This callback returns the travel time between two nodes.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    # The cost of traversing an arc is the travel time.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows and Duration Limit constraint.
    time = "Time"
    # Add a time dimension to the routing model.
    # Arguments:
    #   - transit_callback_index: The callback to calculate time between nodes.
    #   - slack_max: Maximum waiting time allowed at a node. A large value (100000)
    #                is used to allow waiting if not explicitly limited.
    #   - capacity: Maximum cumulative value for the dimension (total time for a route).
    #               This enforces the duration_limit for each vehicle's route.
    #   - fix_start_cumul_to_zero: False, as depots might have time windows not starting at 0.
    #   - name: Name of the dimension.
    routing.AddDimension(
        transit_callback_index,
        100000,  # slack_max: A large value to allow waiting
        duration_limit,  # capacity: maximum time per vehicle route
        False,           # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for all locations (customers and depots).
    # Iterate over all possible node indices to ensure all locations are covered.
    # This assumes time_windows list is indexed by location_idx and covers all locations.
    for location_idx in range(len(time_matrix)): # Changed this line
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_windows[location_idx][0], time_windows[location_idx][1])

    # Add time window constraints for each vehicle's start and end nodes (depots).
    # This is crucial for multiple depots, as each vehicle has a specific start/end depot.
    # This part is technically redundant if the above loop covers all nodes correctly,
    # but it explicitly sets constraints for the route start/end points, which can be helpful.
    for vehicle_id in range(num_vehicles):
        # Start node time window
        start_index = routing.Start(vehicle_id)
        start_depot_node = manager.IndexToNode(start_index)
        time_dimension.CumulVar(start_index).SetRange(
            time_windows[start_depot_node][0], time_windows[start_depot_node][1]
        )
        # End node time window
        end_index = routing.End(vehicle_id)
        end_depot_node = manager.IndexToNode(end_index)
        time_dimension.CumulVar(end_index).SetRange(
            time_windows[end_depot_node][0], time_windows[end_depot_node][1]
        )

    # Instantiate route start and end times to produce feasible times.
    # This helps the solver find solutions by minimizing the cumulative time at start/end nodes.
    for i in range(num_vehicles):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # Set a time limit for the solver to prevent it from running indefinitely.
    search_parameters.time_limit.seconds = 30  # Example: 30 seconds time limit

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        obj = solution.ObjectiveValue()

    return obj
