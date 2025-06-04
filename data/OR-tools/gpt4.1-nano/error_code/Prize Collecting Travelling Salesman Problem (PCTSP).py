# Prize Collecting Travelling Salesman Problem (PCTSP)
# This implementation aims to maximize the collected prizes while respecting the maximum travel distance.
# Import necessary modules from OR-Tools.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, prizes: list, max_distance: int, depot: int):
    # Create the routing index manager, which manages the conversion between node indices and routing indices.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, depot)

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes.
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc in the route to be the distance between nodes.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to keep track of the total travel distance.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack, so the distance is strictly the sum of arc costs
        max_distance,  # maximum travel distance allowed
        True,  # start cumul to zero, meaning the route starts with zero distance
        'Distance')

    # Retrieve the distance dimension to access its properties.
    distance_dimension = routing.GetDimensionOrDie('Distance')

    # Add disjunctions for nodes to allow skipping nodes with a penalty equal to their prize.
    for node in range(len(distance_matrix)):
        if node != depot:
            # Penalty for skipping the node is the prize value; dropping the node yields this penalty.
            routing.AddDisjunction([manager.NodeToIndex(node)], prizes[node])

    # Set search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC heuristic for the initial solution.
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Use guided local search for improved solutions.
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Set a time limit of 15 seconds for the solver.
    search_parameters.time_limit.FromSeconds(15)

    # Solve the problem with the specified parameters.
    assignment = routing.SolveWithParameters(search_parameters)

    # Initialize total prize collected.
    total_prize = 0
    if assignment:
        # Start from the beginning of the route.
        index = routing.Start(0)
        # Traverse the route until the end.
        while not routing.IsEnd(index):
            # Convert routing index to node index.
            node = manager.IndexToNode(index)
            # Add the prize of the current node.
            total_prize += prizes[node]
            # Move to the next node in the route.
            next_index = assignment.Value(routing.NextVar(index))
            index = next_index
        # Return the total prize collected.
        return total_prize
    # If no solution is found, return 0.
    return 0