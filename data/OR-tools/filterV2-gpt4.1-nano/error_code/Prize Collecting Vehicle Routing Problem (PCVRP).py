# Prize Collecting Vehicle Routing Problem (PCVRP)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, prizes: list, max_distance: int, num_vehicle: int, depot: int):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        # Convert routing variable indices to node indices.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the distance between the nodes from the distance matrix.
        return distance_matrix[from_node][to_node]

    # Register the callback to get the transit (distance) between nodes.
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc (edge) in the routing problem to the distance callback.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to enforce maximum travel distance constraint per vehicle.
    routing.AddDimension(
        transit_callback_index,  # Transit callback index.
        0,  # No slack (additional waiting time allowed).
        max_distance,  # Maximum travel distance per vehicle.
        True,  # Start cumul to zero (initial distance is zero at start).
        'Distance')  # Dimension name.

    # Add disjunctions for prize collection with high penalties.
    # This encourages the solver to include nodes with higher prizes.
    penalty_multiplier = 1000  # A large number to prioritize prize collection.
    for node in range(1, len(distance_matrix)):
        # Calculate penalty based on prize value for each node.
        penalty = prizes[node] * penalty_multiplier
        # Add disjunction with penalty to incentivize visiting high-prize nodes.
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC heuristic for initial solution.
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Use GUIDED_LOCAL_SEARCH for local search metaheuristic.
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Set a time limit of 15 seconds for the solver.
    search_parameters.time_limit.FromSeconds(15)

    # Solve the problem with the specified search parameters.
    assignment = routing.SolveWithParameters(search_parameters)

    total_prize = 0
    # If a solution is found, calculate total prize collected.
    if assignment:
        for node in range(1, len(distance_matrix)):
            # Convert node index to routing variable index.
            index = manager.NodeToIndex(node)
            # Check if the node is visited in the solution.
            if assignment.Value(routing.NextVar(index)) != index:
                # Add the prize of the visited node to total prize.
                total_prize += prizes[node]
    # Return the total prize collected.
    return total_prize