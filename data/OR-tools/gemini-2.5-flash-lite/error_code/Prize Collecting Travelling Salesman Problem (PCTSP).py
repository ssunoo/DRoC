
# Prize Collecting Travelling Salesman Problem (PCTSP)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    num_nodes = len(distance_matrix)
    num_vehicles = 1  # PCTSP implies a single vehicle

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        num_nodes,
        num_vehicles,
        depot)

    # Create routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to limit vehicle travel distance.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)

    # Allow to drop nodes (prize collecting aspect).
    # The depot is typically not dropped in TSP/VRP and has no prize penalty.
    for node_idx in range(num_nodes):
        if node_idx == depot:
            continue  # Depot cannot be dropped

        # Add a disjunction for each node, allowing it to be dropped with a penalty equal to its prize.
        # This penalty is added to the objective if the node is not visited.
        # The objective is minimized, so dropping a node adds its prize to the cost.
        # This effectively minimizes (travel_cost + sum_of_dropped_prizes).
        routing.AddDisjunction(
            [manager.NodeToIndex(node_idx)],
            prizes[node_idx])  # Penalty for dropping the node

    # Setting first solution heuristic and local search metaheuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)

    # Add a time limit to prevent indefinite running, addressing potential timeout issues.
    search_parameters.time_limit.FromSeconds(60)  # Set a reasonable time limit, e.g., 60 seconds

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    obj = None  # Initialize obj to None
    if assignment:
        obj = assignment.ObjectiveValue()

    return obj
