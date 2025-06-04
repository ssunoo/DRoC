
# Prize Collecting Vehicle Routing Problem with Multiple Depots (PCVRPMD)
# [('user', 'The solution failed the code execution test: Error: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmp0bbyohz3.py", line 110, in <module>\n    result = solve(distance_matrix, num_vehicle, starts, ends, prizes, max_distance)\n  File "/tmp/tmp0bbyohz3.py", line 20, in solve\n    manager = pywrapcp.RoutingIndexManager(\n  File "/disks/local/r.2t_6/r13922049/miniconda3/envs/AI/lib/python3.10/site-packages/ortools/constraint_solver/pywrapcp.py", line 4749, in __init__\n    _pywrapcp.RoutingIndexManager_swiginit(self, _pywrapcp.new_RoutingIndexManager(*args))\nTypeError: Wrong number or type of arguments for overloaded function \'new_RoutingIndexManager\'.\n  Possible C/C++ prototypes are:\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,operations_research::RoutingIndexManager::NodeIndex)\n    operations_research::RoutingIndexManager::RoutingIndexManager(int,int,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &,std::vector< operations_research::RoutingIndexManager::NodeIndex > const &)\n\n\n')]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(distance_matrix: list, prizes: list, max_distance: int, num_vehicle: int, starts: list, ends: list):
    """
    Args:
        distance_matrix: contains the integer distance between customers
        prizes: the value of prize that a vehicle can collect at each node
        max_distance: maximum distance that a vehicle can travel
        num_vehicle: the number of the vehicle
        starts: the index of the starting depot for vehicles
        ends: the index of the ending depot for vehicles

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    # Create the routing index manager.
    # The error message suggests an issue with the arguments passed to RoutingIndexManager.
    # The correct usage depends on the ortools version, but the error indicates that the provided arguments are incorrect.
    # The most common constructor takes the number of nodes, the number of vehicles, and the start and end nodes.
    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix), num_vehicle, starts, ends)

    # Create Routing Model.
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

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # No slack
        max_distance,  # Vehicle maximum travel distance
        True,  # Start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # Optionally, add a cost coefficient for the global span of the dimension.
    # This penalizes longer routes.
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add prize collecting (penalties for unvisited nodes).
    # The solver minimizes total cost, which is travel cost + sum of penalties for unvisited nodes.
    # To maximize collected prizes, we set a penalty for not visiting a node equal to its prize.
    # The objective will then be: sum(prizes_collected) - total_travel_cost.
    # This is equivalent to: total_prizes_possible - (sum(prizes_not_collected) + total_travel_cost).
    # The solver minimizes (sum(prizes_not_collected) + total_travel_cost).
    # So, our desired objective is total_prizes_possible - solver_objective_value.

    total_prizes_possible = 0
    # Iterate through all nodes to identify customer nodes and sum their prizes.
    for node_index in range(len(distance_matrix)):
        # Check if the node is not a starting or ending depot for any vehicle.
        # Depots are mandatory visit points for their assigned vehicles and typically do not have collectable prizes.
        if node_index not in starts and node_index not in ends:
            penalty = prizes[node_index]
            if penalty > 0:  # Only add disjunction if there's a prize to collect
                # Add a disjunction for the node, meaning it can either be visited or skipped with a penalty.
                routing.AddDisjunction([manager.NodeToIndex(node_index)], penalty)
            total_prizes_possible += prizes[node_index]

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(30)  # Set a time limit for solving

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        # The OR-Tools solver minimizes (total_travel_cost + sum_of_penalties_for_unvisited_nodes).
        # Our desired objective is (sum_of_prizes_collected - total_travel_cost).
        # This is equivalent to (total_prizes_possible - (sum_of_penalties_for_unvisited_nodes + total_travel_cost)).
        # Therefore, the objective value is total_prizes_possible minus the solver's objective value.
        obj = total_prizes_possible - solution.ObjectiveValue()
    else:
        obj = -1  # Indicate no solution found

    return obj
