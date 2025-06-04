# Vehicle Routing with Pickups and Deliveries and Distance Limit (PDPL)
# This script solves a vehicle routing problem with pickup and delivery constraints, considering a maximum distance limit per vehicle.
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, distance_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing model indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)
    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert routing indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the distance between the nodes from the distance matrix
        return distance_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc (edge) to the distance callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to track the total distance traveled by each vehicle
    distance_dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack (additional waiting time allowed)
        distance_limit,  # Maximum distance per vehicle
        True,  # Set the start cumul to zero at the beginning of each route
        distance_dimension_name)  # Name of the dimension
    # Retrieve the distance dimension to set additional parameters
    distance_dimension = routing.GetDimensionOrDie(distance_dimension_name)
    # Set a cost coefficient to minimize the maximum route distance (span)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add pickup and delivery constraints for each pair
    for pickup_node, delivery_node in pickups_deliveries:
        # Convert node indices to routing indices
        pickup_idx = manager.NodeToIndex(pickup_node)
        delivery_idx = manager.NodeToIndex(delivery_node)
        # Add pickup and delivery pair constraints
        routing.AddPickupAndDelivery(pickup_idx, delivery_idx)
        # Ensure pickup and delivery are assigned to the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_idx) == routing.VehicleVar(delivery_idx))
        # Enforce that pickup occurs before delivery in the route
        routing.solver().Add(distance_dimension.CumulVar(pickup_idx) <= distance_dimension.CumulVar(delivery_idx))

    # Set the search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC heuristic for the initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute the maximum route distance among all vehicles
    if solution:
        max_route_distance = 0
        # Iterate over each vehicle to calculate its route distance
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            route_distance = 0
            # Traverse the route until the end
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                # Convert indices back to nodes
                node_from = manager.IndexToNode(previous_index)
                node_to = manager.IndexToNode(index)
                # Accumulate the distance for the route
                route_distance += distance_matrix[node_from][node_to]
            # Track the maximum route distance
            max_route_distance = max(max_route_distance, route_distance)
        # Return the maximum route distance found
        return max_route_distance
    else:
        # Return -1 if no solution exists
        return -1