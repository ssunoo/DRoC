# Vehicle Routing with Pickups and Deliveries, Time Windows, Service Time, and Duration Limit (PDPTWSL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        # Convert from routing variable indices to node indices.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return travel time between nodes from the time matrix.
        return time_matrix[from_node][to_node]

    # Register the transit callback to be used for arc costs.
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc to the transit callback.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension to model to handle time windows and duration constraints.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time at nodes
        duration_limit,  # maximum route duration for each vehicle
        False,  # Don't force start cumul to zero.
        time)
    # Retrieve the time dimension for further constraints.
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location except depot.
    for location_idx, window in enumerate(time_windows):
        if location_idx == depot:
            continue
        # Convert node index to routing index.
        index = manager.NodeToIndex(location_idx)
        # Set the time window for the node.
        time_dimension.CumulVar(index).SetRange(window[0], window[1])

    # Set depot time window for all vehicles.
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        # Set the time window for the depot.
        time_dimension.CumulVar(index).SetRange(time_windows[depot][0], time_windows[depot][1])

    # Add pickup and delivery constraints.
    for pickup_node, delivery_node in pickups_deliveries:
        # Convert pickup and delivery nodes to routing indices.
        pickup_index = manager.NodeToIndex(pickup_node)
        delivery_index = manager.NodeToIndex(delivery_node)
        # Add pickup and delivery pair to the routing model.
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure that pickup and delivery are assigned to the same vehicle.
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        # Enforce that pickup occurs before delivery in time.
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Create interval variables for service times at each node.
    service_intervals = []
    for node_idx, service in enumerate(service_time):
        # Convert node index to routing index.
        index = manager.NodeToIndex(node_idx)
        # Get the start variable for the node's time.
        start_var = time_dimension.CumulVar(index)
        # Create a fixed duration interval variable for service time.
        interval_var = routing.solver().FixedDurationIntervalVar(
            start_var, service, f'Service_{node_idx}')
        # Store the interval variable for later constraints.
        service_intervals.append(interval_var)

    # Add precedence constraints: pickup before delivery.
    for pickup_node, delivery_node in pickups_deliveries:
        # Convert pickup and delivery nodes to routing indices.
        pickup_index = manager.NodeToIndex(pickup_node)
        delivery_index = manager.NodeToIndex(delivery_node)
        # Enforce that the pickup interval ends before the delivery interval starts.
        routing.solver().Add(
            service_intervals[pickup_index].EndExpr() <= service_intervals[delivery_index].StartExpr()
        )

    # Set search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC as the first solution strategy.
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Optional: set a time limit for the search (commented out).
    # search_parameters.time_limit.seconds = 30

    # Solve the routing problem with the specified search parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1