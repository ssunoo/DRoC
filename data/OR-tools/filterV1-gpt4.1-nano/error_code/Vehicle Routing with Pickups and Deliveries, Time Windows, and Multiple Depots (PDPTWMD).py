# Vehicle Routing with Pickups and Deliveries, Time Windows, and Multiple Depots (PDPTWMD)
# This code models and solves a vehicle routing problem with pickups, deliveries, time windows, and multiple depots using Google OR-Tools.
# It constructs a routing model, adds constraints for time windows and pickup-delivery pairs, and attempts to find an optimal route.

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'pickups_deliveries': pickups_deliveries,  # Pairs of pickup and delivery nodes
        'num_vehicles': num_vehicle,  # Number of vehicles
        'starts': starts,  # Starting depot indices for each vehicle
        'ends': ends,  # Ending depot indices for each vehicle
    }

    # Create the routing index manager to handle node indices and vehicle routes
    manager = pywrapcp.RoutingIndexManager(
        len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Instantiate the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Define transit callback to return travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    # Register the transit callback with the routing model
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator for all vehicles to minimize travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension for time to incorporate time windows and waiting times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # Allow waiting time at nodes
        30,  # Maximum total time per vehicle
        False,  # Do not force start cumul to zero
        time
    )
    # Retrieve the time dimension to set constraints
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location except depots
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx in data['starts'] or location_idx in data['ends']:
            continue  # Skip depots as they are handled separately
        index = manager.NodeToIndex(location_idx)
        # Set the allowed time window for the location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)
        depot_idx = data['starts'][vehicle_id]
        # Check if depot has a valid time window
        if 0 <= depot_idx < len(data['time_windows']):
            time_window = data['time_windows'][depot_idx]
            # Set the time window for the start node if valid
            if time_window and len(time_window) == 2:
                time_dimension.CumulVar(start_index).SetRange(time_window[0], time_window[1])

    # Add pickup and delivery constraints to ensure correct pairing and order
    for pickup_node, delivery_node in data['pickups_deliveries']:
        pickup_idx = manager.NodeToIndex(pickup_node)
        delivery_idx = manager.NodeToIndex(delivery_node)
        routing.AddPickupAndDelivery(pickup_idx, delivery_idx)
        # Ensure both pickup and delivery are handled by the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_idx) == routing.VehicleVar(delivery_idx))
        # Enforce that pickup occurs before delivery
        routing.solver().Add(
            time_dimension.CumulVar(pickup_idx) <= time_dimension.CumulVar(delivery_idx)
        )

    # Minimize the arrival time at start nodes for all vehicles
    for vehicle_id in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(vehicle_id)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(vehicle_id)))

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use a heuristic to find an initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1