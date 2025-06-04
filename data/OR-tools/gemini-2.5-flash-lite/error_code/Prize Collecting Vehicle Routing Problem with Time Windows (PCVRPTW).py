
# Prize Collecting Vehicle Routing Problem with Time Windows (PCVRPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    num_nodes = len(time_matrix)

    # Create the routing index manager.
    # The depot argument should be a list of start and end nodes, even for a single depot.
    manager = pywrapcp.RoutingIndexManager(
        num_nodes,
        num_vehicle,
        [depot],  # Start depots
        [depot]   # End depots
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time_dimension_name = "Time"
    # A large slack value to allow waiting time.
    # The capacity is the maximum duration a vehicle can travel.
    routing.AddDimension(
        transit_callback_index,
        1000000,  # allow waiting time (slack_max) - a large constant
        max_duration,  # vehicle maximum travel duration (capacity)
        True,  # start cumul to zero
        time_dimension_name
    )
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)

    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue  # Depot handled separately for vehicle start/end
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot).
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )

    # Instantiate route start and end times to produce feasible times.
    # This is crucial for the solver to find a solution and ensures that
    # the end time of the route respects the depot's time window if the
    # vehicle returns to the depot.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Allow to drop nodes (Prize Collecting).
    # The cost of dropping a node is its prize value. The objective will be to
    # minimize (total_travel_time + sum_of_dropped_prizes), which is equivalent
    # to maximizing (sum_of_collected_prizes - total_travel_time) plus a constant.
    for node_idx in range(num_nodes):
        if node_idx == depot:
            continue  # Depot cannot be dropped
        routing.AddDisjunction(
            [manager.NodeToIndex(node_idx)],
            prizes[node_idx]
        )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    # Set a time limit for the solver to prevent indefinite running.
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    obj = None
    # Get the objective value.
    if assignment:
        obj = assignment.ObjectiveValue()

    return obj
