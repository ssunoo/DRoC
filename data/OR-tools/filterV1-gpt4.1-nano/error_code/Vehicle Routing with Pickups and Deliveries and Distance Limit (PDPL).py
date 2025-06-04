# Vehicle Routing with Pickups and Deliveries and Distance Limit (PDPL)
# This code solves a vehicle routing problem with pickup and delivery constraints and a maximum distance limit per vehicle.
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, distance_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)
    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Returns the distance between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc (edge) for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to enforce distance limit per vehicle
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack, no extra distance allowed
        distance_limit,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance_dimension_name)

    # Retrieve the distance dimension to set additional constraints or parameters
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)
    # Set global span cost coefficient to minimize the maximum route distance
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Pickup and Delivery constraints
    for pickup_node, delivery_node in pickups_deliveries:
        # Convert node indices to routing indices
        pickup_index = manager.NodeToIndex(pickup_node)
        delivery_index = manager.NodeToIndex(delivery_node)
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Enforce that pickup and delivery are on the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce that pickup occurs before delivery in the route
        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC heuristic for initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute and return the maximum route distance
    if solution:
        max_distance = 0
        # Iterate over each vehicle to compute its route distance
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            route_distance = 0
            # Traverse the route until the end node
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                # Add the distance of the current arc to the route distance
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            # Track the maximum route distance among all vehicles
            max_distance = max(max_distance, route_distance)
        return max_distance
    else:
        # Return -1 if no solution exists
        return -1