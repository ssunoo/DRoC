# Capacitated Vehicle Routing Problem (CVRP)
# This code solves the CVRP using Google OR-Tools.
# It finds the optimal routes for a fleet of vehicles to deliver demands to various locations,
# minimizing the total distance traveled while respecting vehicle capacity constraints.
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, demands: list, num_vehicle: int, vehicle_capacity: list, depot: int):
    # Create the routing index manager, which manages the conversion between the problem's node indices and the solver's internal indices.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model that will contain the routing problem.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes.
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the distance callback and get its index.
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc (edge) to be the distance between nodes.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Create and register a demand callback for vehicle demands at each node.
    def demand_callback(from_index):
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    # Register the demand callback.
    demand_callback_index = routing.RegisterTransitCallback(demand_callback)
    # Add capacity dimension to ensure vehicle capacities are not exceeded.
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacity,  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Set the search parameters for the solver, using a heuristic for initial solutions.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the routing problem with the specified parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize total distance to zero.
    total_distance = 0
    if solution:
        # Loop through each vehicle to calculate the total distance traveled.
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            # Traverse the route until reaching the end.
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                # Add the distance for each arc in the route.
                total_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            # Add the distance from the last node back to the depot to complete the route.
            last_node_index = index
            total_distance += routing.GetArcCostForVehicle(last_node_index, routing.Start(vehicle_id), vehicle_id)
    else:
        # If no solution is found, total distance remains zero.
        total_distance = 0

    # Return the total distance of the computed routes.
    return total_distance