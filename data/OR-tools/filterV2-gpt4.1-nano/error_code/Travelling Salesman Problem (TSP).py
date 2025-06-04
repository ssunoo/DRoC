# Travelling Salesman Problem (TSP)
# [('user', 'The solution failed the code execution test: Error: 'RoutingModel' object has no attribute 'CostOfVehicle'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpru4q8wnf.py", line 55, in <module>\n    result = solve(distance_matrix, depot)\n  File "/tmp/tmpru4q8wnf.py", line 40, in solve\n    obj = routing.CostOfVehicle(0, solution)\nAttributeError: 'RoutingModel' object has no attribute 'CostOfVehicle'\n\n')}

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, depot: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        depot: the index of the start node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, depot)

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

    # Return the objective value
    if solution:
        # Correct method to get the total cost of the route
        obj = routing.CostOfVehicle(0, solution)
    else:
        obj = -1
    return obj