# Vehicle Routing with Pickups and Deliveries, Service Time, and Duration Limit (PDPSL)
# This code solves a vehicle routing problem with pickups, deliveries, service times, and route duration constraints using Google OR-Tools.
# Import necessary modules from OR-Tools
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Create the data model containing all problem data
    data = {
        'distance_matrix': time_matrix,  # Matrix of travel times/distances between nodes
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

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]  # Return distance between nodes

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles to the distance callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Register and add a distance dimension to track total distance traveled
    distance_callback_index = routing.RegisterTransitCallback(distance_callback)
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        distance_callback_index,  # Transit callback index
        0,  # No slack (waiting time)
        0,  # Maximum distance, set to 0 for now or a large number if needed
        True,  # Force start cumul to zero
        distance_dimension_name)  # Name of the dimension
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)  # Retrieve the dimension

    # Add Pickup and Delivery constraints to ensure pickups occur before deliveries
    for pickup_node, delivery_node in data['pickups_deliveries']:
        pickup_idx = manager.NodeToIndex(pickup_node)  # Convert node to index
        delivery_idx = manager.NodeToIndex(delivery_node)
        routing.AddPickupAndDelivery(pickup_idx, delivery_idx)  # Add pickup-delivery pair
        routing.solver().Add(routing.VehicleVar(pickup_idx) == routing.VehicleVar(delivery_idx))  # Same vehicle for pickup and delivery
        # Add precedence constraint: pickup must occur before delivery
        routing.solver().Add(distance_dimension.CumulVar(pickup_idx) <= distance_dimension.CumulVar(delivery_idx))

    # Create and register a service time callback
    def service_time_callback(from_index):
        node = manager.IndexToNode(from_index)  # Convert index to node
        return data['service_time'][node]  # Return service time at node

    service_time_callback_index = routing.RegisterTransitCallback(service_time_callback)  # Register callback

    # Add a time dimension to account for service times and route duration limits
    time_dimension_name = 'Time'
    routing.AddDimension(
        service_time_callback_index,  # Transit callback index for service times
        0,  # No slack
        data['duration_limit'],  # Maximum route duration
        True,  # Force start cumul to zero
        time_dimension_name)  # Name of the dimension
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)  # Retrieve the dimension

    # Add route duration constraints for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Start node index for vehicle
        end_index = routing.End(vehicle_id)  # End node index for vehicle
        end_time_var = time_dimension.CumulVar(end_index)  # End time variable
        routing.solver().Add(end_time_var <= data['duration_limit'])  # Enforce duration limit

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution
    search_parameters.time_limit.seconds = 30  # Time limit for search

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Total cost of the solution
    else:
        return -1  # No solution found