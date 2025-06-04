
# Prize Collecting Vehicle Routing Problem with Time Windows and Multiple Depots (PCVRPTWMD)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    num_nodes = len(time_matrix)

    # Ensure starts and ends are lists for multiple depots.
    # If they are single integers, convert them to lists of length num_vehicle.
    if not isinstance(starts, list):
        starts = [starts] * num_vehicle
    if not isinstance(ends, list):
        ends = [ends] * num_vehicle

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(num_nodes, num_vehicle, starts, ends)

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

    # Add Time Windows constraint.
    time_dimension_name = "Time"
    routing.AddDimension(
        transit_callback_index,
        max_time,  # allow waiting time (slack) up to max_time
        max_time,  # maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero, as we have multiple depots
        time_dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Instantiate route start and end times to produce feasible times.
    # This helps the solver find feasible solutions faster.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Allow to drop nodes (Prize Collecting).
    # The penalty for dropping a node is its prize value.
    # We want to maximize collected prizes, which is equivalent to minimizing
    # (total possible prizes - collected prizes) = dropped prizes.
    # So, the OR-Tools objective will be total_travel_time + sum(dropped_prizes).
    # The actual objective we want is sum(collected_prizes) - total_travel_time.
    # This can be derived as: TotalPrizes - (OR-Tools Objective).
    all_depots = set(starts + ends)
    for node_idx in range(num_nodes):
        if node_idx not in all_depots: # Only allow dropping non-depot nodes
            routing.AddDisjunction(
                [manager.NodeToIndex(node_idx)],
                prizes[node_idx]
            )

    # Setting first solution heuristic and metaheuristics.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    # Add a time limit to prevent the solver from running indefinitely.
    search_parameters.time_limit.FromSeconds(30) # Set a reasonable time limit

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Calculate the objective value.
    obj = -1 # Default value if no solution is found
    if assignment:
        total_possible_prizes = sum(prizes)
        obj = total_possible_prizes - assignment.ObjectiveValue()

    return obj
