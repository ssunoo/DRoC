# Prize Collecting Travelling Salesman Problem with Time Windows (PCTSPTW)
# This code solves a variant of the TSP where the goal is to maximize collected prizes within time constraints.
# It uses Google OR-Tools to model and solve the problem.
# Import necessary modules from OR-Tools.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, depot: int, prizes: list, max_duration: int):
    # Create the routing index manager.
    # Manages the conversion between node indices and internal routing indices.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, depot)

    # Create routing model.
    # Represents the routing problem.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    # Defines the travel time between nodes.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator.
    # Uses travel time as the cost for routing.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to limit total travel time.
    # Ensures total travel time does not exceed max_duration.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_duration,  # maximum travel time
        True,  # start cumul to zero
        'Time')
    time_dimension = routing.GetDimensionOrDie('Time')

    # Add time window constraints.
    # Ensures each node is visited within its specified time window.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add prize collection as disjunctions with penalty.
    # Nodes can be skipped at a penalty equal to their prize.
    for node_idx, prize in enumerate(prizes):
        if node_idx != depot:
            index = manager.NodeToIndex(node_idx)
            # Penalty is negative prize to maximize total prize.
            routing.AddDisjunction([index], -prize)

    # Set search parameters.
    # Defines how the solver searches for solutions.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    # Finds a route that maximizes prize collection within constraints.
    solution = routing.SolveWithParameters(search_parameters)

    total_prize = 0
    if solution:
        # Extract the route.
        index = routing.Start(0)
        route_nodes = [manager.IndexToNode(index)]  # include start node
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route_nodes.append(manager.IndexToNode(index))
        # Sum prizes for nodes in the route.
        for node_idx in route_nodes:
            if node_idx != depot:
                total_prize += prizes[node_idx]
    return total_prize