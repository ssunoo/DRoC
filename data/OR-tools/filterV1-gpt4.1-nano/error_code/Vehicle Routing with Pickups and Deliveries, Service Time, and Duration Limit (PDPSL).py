# Vehicle Routing with Pickups and Deliveries, Service Time, and Duration Limit (PDPSL)
# This code solves a vehicle routing problem with pickups, deliveries, service times, and route duration constraints
# Import necessary OR-Tools modules
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(time_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Create the data model containing all problem data
    data = {
        'distance_matrix': time_matrix,  # Matrix of travel times between nodes
        'pickups_deliveries': pickups_deliveries,  # List of pickup and delivery node pairs
        'num_vehicles': num_vehicle,  # Number of vehicles available
        'depot': depot,  # Starting depot node index
        'service_time': service_time,  # Service time at each node
        'duration_limit': duration_limit  # Maximum allowed route duration
    }

    # Create the routing index manager to manage node indices for vehicles
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return data['distance_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles to minimize total travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Pickup and Delivery constraints for each pair
    for pickup_node, delivery_node in data['pickups_deliveries']:
        pickup_idx = manager.NodeToIndex(pickup_node)  # Convert pickup node to routing index
        delivery_idx = manager.NodeToIndex(delivery_node)  # Convert delivery node to routing index
        routing.AddPickupAndDelivery(pickup_idx, delivery_idx)  # Add pickup-delivery pair constraint
        # Enforce that pickup and delivery are served by the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_idx) == routing.VehicleVar(delivery_idx))
        # Enforce that pickup occurs before delivery
        # Retrieve the dimension object for 'ServiceTime'
        service_time_dimension = routing.GetDimensionOrDie('ServiceTime')
        # Add constraint: pickup service time must be less than or equal to delivery service time
        routing.solver().Add(service_time_dimension.CumulVar(pickup_idx) <= service_time_dimension.CumulVar(delivery_idx))

    # Add Service Time dimension to account for service durations at each node
    def service_time_callback(from_index):
        node = manager.IndexToNode(from_index)  # Convert routing index to node index
        return data['service_time'][node]  # Return service time at the node

    service_time_callback_index = routing.RegisterTransitCallback(service_time_callback)  # Register callback

    # Add the ServiceTime dimension with no slack, large horizon, and start cumul at zero
    routing.AddDimension(
        service_time_callback_index,
        0,  # No slack time allowed
        sum(data['service_time']) * len(data['service_time']),  # Large enough horizon to cover all service times
        True,  # Force start cumul to zero at the depot
        'ServiceTime')  # Name of the dimension

    # Add Duration Limit dimension to enforce maximum route duration
    def travel_time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return data['distance_matrix'][from_node][to_node]  # Return travel time between nodes

    travel_time_callback_index = routing.RegisterTransitCallback(travel_time_callback)  # Register callback

    # Add the Duration dimension with no slack, maximum duration limit, and start cumul at zero
    routing.AddDimension(
        travel_time_callback_index,
        0,  # No slack time
        data['duration_limit'],  # Maximum route duration
        True,  # Force start cumul to zero
        'Duration')  # Name of the dimension

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC strategy for initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Set a time limit of 30 seconds for the solver
    search_parameters.time_limit.seconds = 30

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1