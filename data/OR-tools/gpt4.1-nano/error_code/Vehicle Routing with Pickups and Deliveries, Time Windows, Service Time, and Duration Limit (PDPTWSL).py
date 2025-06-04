# Vehicle Routing with Pickups and Deliveries, Time Windows, Service Time, and Duration Limit (PDPTWSL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Instantiate the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'pickups_deliveries': pickups_deliveries,
        'num_vehicles': num_vehicle,
        'depot': depot,
        'service_time': service_time,
        'duration_limit': duration_limit
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for travel times
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to node index in the data
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return travel time between nodes
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension to model to handle time windows and duration constraints
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time at nodes
        data['duration_limit'],  # maximum route duration
        False,  # Don't force start cumul to zero
        time
    )
    # Retrieve the time dimension
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        # Set the time window for each node
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Set the depot's time window
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add Pickup and Delivery constraints
    for pickup_node, delivery_node in data['pickups_deliveries']:
        # Convert pickup and delivery nodes to routing indices
        pickup_index = manager.NodeToIndex(pickup_node)
        delivery_index = manager.NodeToIndex(delivery_node)
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure pickup and delivery are assigned to the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce that pickup occurs before delivery
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Add Service Time constraints for each node
    for node_idx, service in enumerate(data['service_time']):
        # Convert node index to routing index
        index = manager.NodeToIndex(node_idx)
        # Add service time as a fixed interval (assuming service is duration)
        routing.solver().Add(
            time_dimension.CumulVar(index) >= time_dimension.CumulVar(index) - service
        )

    # Set the maximum route duration for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Enforce route duration limit at start node
        routing.solver().Add(time_dimension.CumulVar(index) <= data['duration_limit'])

    # Configure search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Set a time limit for the solver
    search_parameters.time_limit.seconds = 30
    # Use a heuristic for initial solution (cheapest arc)
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1
