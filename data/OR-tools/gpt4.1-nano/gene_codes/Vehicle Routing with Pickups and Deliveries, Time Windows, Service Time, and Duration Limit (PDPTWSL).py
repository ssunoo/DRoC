# Vehicle Routing with Pickups and Deliveries, Time Windows, Service Time, and Duration Limit (PDPTWSL)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Instantiate the data model as a dictionary containing all problem data
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'pickups_deliveries': pickups_deliveries,
        'num_vehicles': num_vehicle,
        'depot': depot,
        'service_time': service_time,
        'duration_limit': duration_limit
    }

    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times plus service times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)      # Convert routing index to node index
        travel_time = data['time_matrix'][from_node][to_node]  # Get travel time between nodes
        # Add service time at the from_node
        service_time = data['service_time'][from_node]
        return travel_time + service_time  # Total time including travel and service

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a time dimension to handle time windows and route duration constraints
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        data['duration_limit'],  # Maximum route duration
        False,  # Do not force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        # Set the time window for the start node (depot)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],  # Earliest start time at depot
            data['time_windows'][data['depot']][1]   # Latest start time at depot
        )

    # Add pickup and delivery constraints
    for pickup_node, delivery_node in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(pickup_node)  # Convert pickup node
        delivery_index = manager.NodeToIndex(delivery_node)  # Convert delivery node
        routing.AddPickupAndDelivery(pickup_index, delivery_index)  # Add pickup-delivery pair
        # Ensure pickup and delivery are assigned to the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce that pickup occurs before delivery
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Set the maximum route duration for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index
        # Add constraint for route duration not to exceed the limit
        routing.solver().Add(time_dimension.CumulVar(index) <= data['duration_limit'])

    # Configure search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 30  # Set a time limit for the solver
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1