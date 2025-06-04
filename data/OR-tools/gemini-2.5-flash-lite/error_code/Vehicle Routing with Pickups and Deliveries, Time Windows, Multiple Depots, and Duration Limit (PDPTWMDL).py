
# Vehicle Routing with Pickups and Deliveries, Time Windows, Multiple Depots, and Duration Limit (PDPTWMDL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    num_locations = len(time_matrix)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        num_locations, num_vehicle, starts, ends
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
        duration_limit,  # allow waiting time (slack_max)
        duration_limit,  # maximum time per vehicle (capacity)
        False,           # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start and end node (depots).
    for vehicle_id in range(num_vehicle):
        start_index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        
        start_node = manager.IndexToNode(start_index)
        end_node = manager.IndexToNode(end_index)

        time_dimension.CumulVar(start_index).SetRange(
            time_windows[start_node][0], time_windows[start_node][1]
        )
        time_dimension.CumulVar(end_index).SetRange(
            time_windows[end_node][0], time_windows[end_node][1]
        )

    # Add Pickups and Deliveries constraints.
    for request in pickups_deliveries:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        
        # Same vehicle constraint
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        
        # Pickup before delivery constraint
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Add variable minimized by finalizer to help the solver find feasible times.
    # This is crucial for time window problems as it guides the solver to find optimal start/end times.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

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
    obj = -1  # Initialize obj to a default value in case no solution is found
    if solution:
        obj = solution.ObjectiveValue()

    return obj
