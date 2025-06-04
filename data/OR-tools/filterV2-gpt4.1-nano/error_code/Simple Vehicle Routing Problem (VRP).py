# Simple Vehicle Routing Problem (VRP)
# [('user', 'You solution returns nothing or 0. The program may be incomplete or your solution does not work for the task.')]
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

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the total distance of the solution
    obj = 0
    if solution:
        index = routing.Start(0)
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            obj += routing.GetArcCostForVehicle(previous_index, index, 0)
    else:
        obj = -1
    return obj