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
        # Returns the distance between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # If solution exists, compute total distance
    obj = -1
    if solution:
        total_distance = 0
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                # Accumulate the distance for each arc in the route
                total_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        # Set the objective as the total distance
        obj = total_distance
    return obj