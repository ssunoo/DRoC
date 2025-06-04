# Prize Collecting Travelling Salesman Problem (PCTSP)
# This code solves the PCTSP using Google OR-Tools.
# It finds a route starting and ending at the depot that maximizes the total prizes collected,
# while respecting the maximum travel distance constraint.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, prizes: list, max_distance: int, depot: int):
    '''
    Args:
        distance_matrix: contains the integer distance between customers
        prizes: the value of prize that a vehicle can collect at each node
        max_distance: maximum distance that a vehicle can travel
        depot: the index of the depot node

    Returns:
        obj: a number representing the objective value of the solution
    '''
    # Create the routing index manager.
    # Manages the conversion between the node indices and the internal routing indices.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, depot)

    # Create routing model.
    # Represents the routing problem.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    # This callback returns the distance between two nodes.
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc.
    # This tells the solver to minimize the total distance.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    # Ensures the total travel distance does not exceed max_distance.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,  # maximum travel distance
        True,  # start cumul to zero
        'Distance')

    # Retrieve the distance dimension.
    distance_dimension = routing.GetDimensionOrDie('Distance')

    # Add prize collection as node attribute.
    # Create a callback to return the prize value at each node.
    def prize_callback(node_index):
        node = manager.IndexToNode(node_index)
        return prizes[node]

    prize_callback_index = routing.RegisterUnaryTransitCallback(prize_callback)

    # Add a dimension for prizes with zero slack, no capacity, and maximize.
    # This dimension accumulates prizes collected along the route.
    routing.AddDimension(
        prize_callback_index,
        0,  # no slack
        sum(prizes),  # upper bound for total prizes
        True,  # start cumul to zero
        'Prize')

    # Retrieve the prize dimension.
    prize_dimension = routing.GetDimensionOrDie('Prize')

    # Set the objective to maximize total prizes.
    # Configure search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use a heuristic to find an initial solution.
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Use a local search metaheuristic to improve the solution.
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Set a time limit for the solver.
    search_parameters.time_limit.FromSeconds(15)
    # Enable logging of the search process.
    search_parameters.log_search = True
    # Limit the number of solutions considered.
    search_parameters.solution_limit = 1

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    total_prize = 0
    if assignment:
        # Start from the depot.
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            # Accumulate prize only if node is not depot.
            if node != depot:
                total_prize += prizes[node]
            # Move to the next node in the route.
            index = assignment.Value(routing.NextVar(index))
    return total_prize
