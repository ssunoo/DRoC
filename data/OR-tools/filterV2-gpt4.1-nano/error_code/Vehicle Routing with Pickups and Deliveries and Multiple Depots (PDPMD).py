# Vehicle Routing with Pickups and Deliveries and Multiple Depots (PDPMD)
# This code solves a vehicle routing problem with pickups and deliveries across multiple depots using Google OR-Tools.
# Import necessary modules from OR-Tools
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(distance_matrix: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list):
    # Create the routing index manager to manage the conversion between node indices and routing variables
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts, ends)
    # Instantiate the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable index to actual node index in the distance matrix
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc (edge) to be the distance as defined by the callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add pickup and delivery constraints to ensure correct pairing and order
    for pickup, delivery in pickups_deliveries:
        # Convert pickup and delivery nodes to routing indices
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        # Add pickup and delivery pair constraints
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure that the pickup and delivery are assigned to the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # The AddPickupAndDelivery method enforces that pickup occurs before delivery
        # No need to add additional constraints for order here

    # Set the search parameters for the solver, choosing a heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute and return the total distance
    if solution:
        total_distance = solution.Value(routing.CostVar())
        return total_distance
    else:
        # Return -1 if no solution is found
        return -1