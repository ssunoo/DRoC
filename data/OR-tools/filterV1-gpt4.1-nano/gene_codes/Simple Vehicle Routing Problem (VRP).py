# Simple Vehicle Routing Problem (VRP)
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve(distance_matrix: list, num_vehicle: int, depot: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        num_vehicle: the number of the vehicle
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc using the registered callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the VRP problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, calculate the total distance of the routes
    if solution:
        total_distance = 0
        # Loop through each vehicle to compute its route distance
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            # Traverse the route until reaching the end node
            while not routing.IsEnd(index):
                previous_index = index
                # Move to the next node in the route
                index = solution.Value(routing.NextVar(index))
                # Add the arc cost for the current segment
                total_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        obj = total_distance  # Total distance as the objective value
    else:
        obj = -1  # Return -1 if no solution is found
    return obj