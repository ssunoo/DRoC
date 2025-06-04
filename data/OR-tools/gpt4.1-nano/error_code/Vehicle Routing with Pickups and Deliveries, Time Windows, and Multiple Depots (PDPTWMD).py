# Vehicle Routing with Pickups and Deliveries, Time Windows, and Multiple Depots (PDPTWMD)
# This code models and solves a vehicle routing problem with pickups, deliveries, time windows, and multiple depots using Google OR-Tools.
# It constructs a routing model, adds constraints, and attempts to find an optimal route minimizing total travel time.

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between nodes
        'time_windows': time_windows,  # Allowed time windows for each node
        'pickups_deliveries': pickups_deliveries,  # List of pickup-delivery pairs
        'num_vehicles': num_vehicle,  # Number of vehicles
        'starts': starts,  # Start nodes for each vehicle
        'ends': ends,  # End nodes for each vehicle
        'depot': 0  # Assuming the first node as depot if not specified
    }

    # Create the routing index manager to handle node indices and vehicle routes
    manager = pywrapcp.RoutingIndexManager(
        len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Instantiate the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to return travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator for all vehicles to minimize travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension for time to handle time windows and waiting times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # Allow waiting time at nodes
        30,  # Maximum time per vehicle route
        False,  # Do not force start cumul to zero
        time
    )
    # Retrieve the time dimension to add constraints
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for all nodes
    for location_idx, time_window in enumerate(data['time_windows']):
        if time_window:
            index = manager.NodeToIndex(location_idx)
            start, end = time_window
            # Only set range if start is less than or equal to end
            if start <= end:
                time_dimension.CumulVar(index).SetRange(start, end)

    # Set time window constraints for each vehicle start node based on depot info
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)
        depot_idx = data['starts'][vehicle_id]
        if 0 <= depot_idx < len(data['time_windows']):
            depot_time_window = data['time_windows'][depot_idx]
            if depot_time_window:
                start, end = depot_time_window
                if start <= end:
                    time_dimension.CumulVar(start_index).SetRange(start, end)

    # Add pickup and delivery constraints to ensure correct pairing and order
    for request in data['pickups_deliveries']:
        pickup_node = request[0]
        delivery_node = request[1]
        pickup_index = manager.NodeToIndex(pickup_node)
        delivery_index = manager.NodeToIndex(delivery_node)
        # Enforce pickup and delivery to be on the same vehicle
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce pickup to occur before delivery
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Add time window constraints for depot nodes for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)
        depot_idx = data['starts'][vehicle_id]
        if 0 <= depot_idx < len(data['time_windows']):
            depot_time_window = data['time_windows'][depot_idx]
            if depot_time_window:
                start, end = depot_time_window
                if start <= end:
                    time_dimension.CumulVar(start_index).SetRange(start, end)

    # Minimize the arrival times at start and end nodes for all vehicles
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1