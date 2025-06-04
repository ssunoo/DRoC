
# Prize Collecting Vehicle Routing Problem (PCVRP)
# [('user', 'The solution failed the code execution test: Error: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpxgqlgjpg.py", line 92, in <module>\n    result = solve(distance_matrix, prizes, max_distance, num_vehicle, depot)\n  File "/tmp/tmpxgqlgjpg.py", line 15, in solve\n    manager = pywrapcp.RoutingIndexManager(\n  File "/disks/local/r.2t_6/r13922049/miniconda3/envs/AI/lib/python3.10/site-packages/ortools/constraint_solver/pywrapcp.py", line 4749, in __init__\n    _pywrapcp.RoutingIndexManager_swiginit(self, _pywrapcp.new_RoutingIndexManager(*args))\nTypeError: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\n\n')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
def solve(
    distance_matrix: list[list[int]],
    num_vehicle: int,
    depot: int,
    max_distance: int,
    prizes: list[int]
) -> int:
    """Solves the Prize Collecting Vehicle Routing Problem (PCVRP)."""

    num_nodes = len(distance_matrix)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
            num_nodes,  # Number of nodes
            num_vehicle,  # Number of vehicles
            depot)  # Index of depot node

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

    # Limit Vehicle distance.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)

    # Allow to drop nodes (Prize Collecting).
    # The prize value is used as the penalty for dropping a node.
    # The depot node should not be droppable.
    for node_idx in range(num_nodes):
        if node_idx == depot:
            continue # Depot cannot be dropped

        # AddDisjunction adds a constraint that a node is either visited or dropped.
        # If dropped, a penalty equal to its prize value is incurred.
        # The solver minimizes the total travel cost plus the sum of penalties for dropped nodes.
        routing.AddDisjunction(
                [manager.NodeToIndex(node_idx)],
                prizes[node_idx]) # Use the prize as the penalty for dropping

    # Setting first solution heuristic and local search metaheuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(15) # Set a time limit for solving

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if assignment:
        obj = assignment.ObjectiveValue()
    else:
        obj = -1 # Indicate no solution found

    return obj
