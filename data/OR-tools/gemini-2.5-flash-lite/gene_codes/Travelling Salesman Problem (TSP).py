# Travelling Salesman Problem (TSP)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, depot: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        depot: the index of the start node

    Returns:
        obj: a number representing the objective value of the solution
    """
    num_locations = len(distance_matrix)  # Determine the number of locations from the distance matrix
    num_vehicles = 1  # For TSP, there is only one vehicle

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)  # Manages the indices for the routing model

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)  # Creates the routing model using the index manager

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to user NodeIndex.
        from_node = manager.IndexToNode(from_index)  # Converts the routing variable index to the node index
        to_node = manager.IndexToNode(to_index)  # Converts the routing variable index to the node index
        return distance_matrix[from_node][to_node]  # Returns the distance between two nodes

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)  # Registers the distance callback

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)  # Sets the cost for each arc using the transit callback

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()  # Creates default search parameters
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )  # Sets the first solution strategy to PATH_CHEAPEST_ARC

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)  # Solves the routing problem with the given parameters

    obj = -1  # Default value if no solution is found

    if assignment:
        obj = assignment.ObjectiveValue()  # Gets the objective value from the solution

    return obj  # Returns the objective value
