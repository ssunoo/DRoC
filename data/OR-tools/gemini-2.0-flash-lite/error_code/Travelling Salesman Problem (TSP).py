
# Travelling Salesman Problem (TSP)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve_tsp(distance_matrix, depot):
    """Solves the Traveling Salesman Problem (TSP) using OR-Tools.

    Args:
        distance_matrix: A list of lists representing the distance matrix.
                         distance_matrix[i][j] is the distance from node i to node j.
        depot: The index of the depot (starting and ending node).

    Returns:
        The objective value (total distance of the tour) or -1 if no solution is found.
    """
    # Create the routing index manager.
    num_locations = len(distance_matrix)
    num_vehicles = 1  # Assuming one salesman
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        return solution.ObjectiveValue()
    else:
        return -1
