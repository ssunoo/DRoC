
# Vehicle Routing with Pickups and Deliveries and Time Windows (PDPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicle, depot, depot)

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the travel time between two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)
    time_window = "TimeWindow"
    routing.AddDimension(time_callback_index, 3600 * 24, 3600 * 24, False, time_window) # Add a time dimension to the model.
    time_dimension = routing.GetDimensionOrDie(time_window)
    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1]) # Set the time window for each location.
    # Add time window constraints for the depot.
    depot_idx = depot
    index = manager.NodeToIndex(depot_idx)
    time_dimension.CumulVar(index).SetRange(time_windows[depot_idx][0], time_windows[depot_idx][1]) # Set the time window for the depot.

    # Add pickup and delivery.
    for request in pickups_deliveries:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index) # Add pickup and delivery constraints.
        routing.solver().Add(time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)) # Ensure that the delivery happens after the pickup.

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        obj = solution.ObjectiveValue()
    return obj
