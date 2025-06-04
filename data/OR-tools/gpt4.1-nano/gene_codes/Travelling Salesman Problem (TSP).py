# Travelling Salesman Problem (TSP)
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
    # Create the routing index manager, which manages the conversion between the problem's node indices and the solver's internal indices
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, depot)

    # Create Routing Model, which encapsulates the routing problem
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback, which returns the distance between two nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    # Register the callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc (edge) in the route using the callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic to PATH_CHEAPEST_ARC for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize total distance of the route
    obj = 0
    if solution:
        # Start from the start node
        index = routing.Start(0)
        # Traverse the route until the end
        while not routing.IsEnd(index):
            previous_index = index
            # Move to the next node in the route
            index = solution.Value(routing.NextVar(index))
            # Accumulate the distance for the arc
            obj += routing.GetArcCostForVehicle(previous_index, index, 0)
    else:
        # If no solution is found, set objective to -1
        obj = -1
    # Return the total distance of the route
    return obj