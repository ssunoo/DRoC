# Vehicle Routing with Pickups and Deliveries, Service Time, and Duration Limit (PDPSL)
# This code solves a vehicle routing problem with pickups, deliveries, service times, and route duration constraints
# Import necessary modules from OR-Tools
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(time_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Create the data model as a dictionary to store problem data
    data = {
        'distance_matrix': time_matrix,  # Matrix of travel times between nodes
        'pickups_deliveries': pickups_deliveries,  # List of pickup and delivery node pairs
        'num_vehicles': num_vehicle,  # Number of vehicles available
        'depot': depot,  # Starting and ending point for all vehicles
        'service_time': service_time,  # Service time at each node
        'duration_limit': duration_limit  # Maximum allowed route duration
    }

    # Create the routing index manager to manage node indices for vehicles
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to return travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]  # Return travel time

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles to the travel time callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Pickup and Delivery constraints to ensure correct pairing and order
    for pickup_node, delivery_node in data['pickups_deliveries']:
        pickup_idx = manager.NodeToIndex(pickup_node)  # Convert pickup node to index
        delivery_idx = manager.NodeToIndex(delivery_node)  # Convert delivery node to index
        routing.AddPickupAndDelivery(pickup_idx, delivery_idx)  # Add pickup-delivery pair
        # Enforce that pickup and delivery are served by the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_idx) == routing.VehicleVar(delivery_idx))
        # Enforce that pickup occurs before delivery
        time_dimension = routing.GetDimensionOrDie('Time')  # Get the time dimension
        pickup_var = time_dimension.CumulVar(pickup_idx)  # Cumulative time at pickup
        delivery_var = time_dimension.CumulVar(delivery_idx)  # Cumulative time at delivery
        routing.solver().Add(pickup_var <= delivery_var)  # Pickup must occur before delivery

    # Add a time dimension to model travel times and route durations
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack time
        data['duration_limit'],  # Maximum route duration
        True,  # Force start cumul to zero at the depot
        'Time')  # Name of the dimension
    time_dimension = routing.GetDimensionOrDie('Time')  # Retrieve the time dimension

    # Create and register a callback for service times at each node
    def service_time_callback(from_index):
        node = manager.IndexToNode(from_index)  # Convert index to node
        return data['service_time'][node]  # Return service time for the node

    service_time_callback_index = routing.RegisterUnaryTransitCallback(service_time_callback)  # Register callback

    # Add a dimension for service times
    routing.AddDimension(
        service_time_callback_index,  # Service time callback index
        0,  # No slack
        data['duration_limit'],  # Max cumulative service time
        False,  # Do not force start cumul to zero
        'ServiceTime')  # Name of the dimension
    service_time_dimension = routing.GetDimensionOrDie('ServiceTime')  # Retrieve the service time dimension

    # Initialize route start times for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Start node index for vehicle
        time_dimension.CumulVar(index).SetValue(0)  # Set start time to zero

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 30  # Limit search to 30 seconds
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Return total cost of the solution
    else:
        return -1  # No solution found