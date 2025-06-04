# Vehicle Routing with Pickups and Deliveries (PDP)
# Import necessary modules from OR-Tools
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int):
    # Create the routing index manager to manage the conversion between node indices and routing variables
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc to the transit callback (distance)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Pickup and Delivery constraints for each pair
    for pickup, delivery in pickups_deliveries:
        # Convert pickup and delivery nodes to routing indices
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure the same vehicle handles both pickup and delivery
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce pickup occurs before delivery by comparing cumulative distance variables
        distance_dimension = routing.GetDimensionOrDie('Distance')
        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Set the search parameters with a heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute and return the total distance
    if solution:
        distance_dimension = routing.GetDimensionOrDie('Distance')
        total_distance = solution.ObjectiveValue()
        return total_distance
    else:
        # Return -1 if no solution is found
        return -1