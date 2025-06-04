# Simple Vehicle Routing Problem (VRP)
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, num_vehicle: int, depot: int):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        # Returns the distance between the two nodes.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the callback and get its index.
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc using the registered callback.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the VRP problem with the specified search parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize total distance to zero.
    total_distance = 0
    if solution:
        # Loop through each vehicle to calculate total distance.
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            # Traverse the route until the end node.
            while not routing.IsEnd(index):
                previous_index = index
                # Move to the next node in the route.
                index = solution.Value(routing.NextVar(index))
                # Add the distance of the arc to total distance.
                total_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
    else:
        # If no solution is found, set total distance to -1.
        total_distance = -1

    # Return the total distance of the computed routes.
    return total_distance