# Prize Collecting Travelling Salesman Problem (PCTSP)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, prizes: list, max_distance: int, depot: int):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        prizes: the value of prize that a vehicle can collect at each node
        max_distance: maximum distance that a vehicle can travel
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, depot)

    # Create routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,  # maximum travel distance
        True,  # start cumul to zero
        'Distance')

    distance_dimension = routing.GetDimensionOrDie('Distance')

    # Add disjunctions with penalties to allow skipping nodes.
    for node in range(1, len(distance_matrix)):
        routing.AddDisjunction([manager.NodeToIndex(node)], prizes[node])

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(15)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    total_prize = 0
    if assignment:
        index = routing.Start(0)
        visited_nodes = set()
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            visited_nodes.add(node)
            index = assignment.Value(routing.NextVar(index))
        # Add prize for all visited nodes.
        total_prize = sum(prizes[node] for node in visited_nodes)
        return total_prize
    return 0