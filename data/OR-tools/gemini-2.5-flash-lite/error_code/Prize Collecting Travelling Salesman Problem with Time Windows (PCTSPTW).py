
# Prize Collecting Travelling Salesman Problem with Time Windows (PCTSPTW)
# [('user', 'You solution returns nothing or 0. The program may be incomplete or your solution does not work for the task.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve(time_matrix, time_windows, depot, prizes, max_duration):
    """Solves the Prize Collecting Traveling Salesman Problem with Time Windows (PCTSPTW).

    Args:
        time_matrix: A 2D list representing the travel times between nodes.
        time_windows: A list of tuples, where each tuple represents the time window (start, end) for a node.
        depot: The index of the depot node.
        prizes: A list of prizes for each node.
        max_duration: The maximum allowed travel duration for the vehicle.

    Returns:
        The objective value (total collected prizes - travel cost) if a solution is found, -1 otherwise.
    """
    num_nodes = len(time_matrix)
    num_vehicles = 1  # For TSP, typically one vehicle

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(num_nodes, num_vehicles, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes.

        Args:
            from_index: The index of the starting node in the routing model.
            to_index: The index of the ending node in the routing model.

        Returns:
            The travel time between the two nodes.
        """
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint and Max Duration.
    time_dimension_name = "Time"
    # Slack allows vehicles to wait at a node.
    # Capacity is the maximum duration a vehicle can travel.
    # Setting slack to max_duration allows waiting up to the total route duration.
    routing.AddDimension(
        transit_callback_index,
        max_duration,  # slack (allow waiting time)
        max_duration,  # vehicle maximum travel duration (capacity)
        False,  # Don't force start cumul to zero.
        time_dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start and end node.
    # The depot's time window applies to the start and end of the route.
    for vehicle_id in range(num_vehicles):
        start_index = routing.Start(vehicle_id)
        time_dimension.CumulVar(start_index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )
        end_index = routing.End(vehicle_id)
        time_dimension.CumulVar(end_index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )

    # Instantiate route start and end times to produce feasible times.
    # This helps the solver find a solution by minimizing the start/end times.
    for i in range(num_vehicles):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Allow to drop nodes for Prize Collecting.
    # The penalty for dropping a node is its prize value.
    # The objective will be to minimize (total_travel_cost + sum_of_dropped_prizes).
    # To maximize collected prizes, we want to minimize sum_of_dropped_prizes.
    for node_idx in range(num_nodes):
        if node_idx == depot:
            continue  # Depot cannot be dropped
        # Assuming prizes[depot] is already 0 as per problem context.
        routing.AddDisjunction(
            [manager.NodeToIndex(node_idx)],
            prizes[node_idx]
        )

    # Setting first solution heuristic and local search metaheuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    # Optional: Add a time limit for the solver
    search_parameters.time_limit.FromSeconds(30) # 30 seconds as a reasonable default

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    obj = -1
    if assignment:
        # The OR-Tools objective value is (total_travel_cost + sum_of_dropped_prizes).
        # We want to maximize (total_collected_prizes - total_travel_cost).
        # Let P_total = sum(prizes).
        # Let P_collected = total_collected_prizes.
        # Let P_dropped = sum_of_dropped_prizes.
        # P_total = P_collected + P_dropped => P_dropped = P_total - P_collected.
        # OR-Tools Objective (Z_ortools) = total_travel_cost + P_dropped
        # Z_ortools = total_travel_cost + (P_total - P_collected)
        # We want P_collected - total_travel_cost.
        # P_collected - total_travel_cost = P_total - Z_ortools.
        total_possible_prizes = sum(prizes)
        obj = total_possible_prizes - assignment.ObjectiveValue()

    return obj
