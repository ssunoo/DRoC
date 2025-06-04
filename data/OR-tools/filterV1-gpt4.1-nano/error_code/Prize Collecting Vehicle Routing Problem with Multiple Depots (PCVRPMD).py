# Prize Collecting Vehicle Routing Problem with Multiple Depots (PCVRPMD)
# This code solves a variant of the vehicle routing problem where multiple depots are involved,
# and the goal is to maximize the collected prizes while respecting maximum travel distances.
# It uses Google's OR-Tools routing library.

def solve(distance_matrix: list, prizes: list, max_distance: int, num_vehicle: int, starts, ends):
    # Convert starts and ends to list if they are integers, to handle single depot cases.
    if isinstance(starts, int):
        starts_node_index = [starts]
    else:
        starts_node_index = list(starts)
    
    if isinstance(ends, int):
        ends_node_index = [ends]
    else:
        ends_node_index = list(ends)

    # Create the routing index manager, which manages the conversion between node indices and internal routing indices.
    # The constructor expects the number of nodes, number of vehicles, list of start nodes, and list of end nodes.
    # Ensure that starts_node_index and ends_node_index are correctly formatted as lists.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts_node_index, ends_node_index)

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Define a callback function to return distances between nodes.
    def distance_callback(from_index, to_index):
        # Convert routing indices to node indices.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the distance between the nodes from the distance matrix.
        return distance_matrix[from_node][to_node]

    # Register the distance callback with the routing model.
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the arc cost evaluator for all vehicles to the distance callback.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to enforce maximum travel distance per vehicle.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,  # maximum travel distance per vehicle
        True,  # start cumul to zero
        'Distance')

    # Retrieve the distance dimension to add constraints or analyze.
    distance_dimension = routing.GetDimensionOrDie('Distance')

    # Add prize collection as node penalties to incentivize visiting high-prize nodes.
    for node in range(len(distance_matrix)):
        if node not in starts_node_index and node not in ends_node_index:
            # Penalty for dropping node: total prize (to maximize prize, penalties are negative prizes)
            penalty = prizes[node]
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Set search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use a greedy heuristic to find an initial solution.
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Use guided local search for better solutions.
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Set a time limit for the solver.
    search_parameters.time_limit.FromSeconds(15)

    # Solve the routing problem with the specified parameters.
    assignment = routing.SolveWithParameters(search_parameters)

    # Initialize total prize collected.
    total_prize = 0
    if assignment:
        # Iterate over all nodes in the routing solution.
        for node in range(routing.Size()):
            index = node
            # Skip start and end nodes.
            if routing.IsStart(index) or routing.IsEnd(index):
                continue
            # Check if the node is visited in the solution.
            if assignment.Value(routing.NextVar(index)) != index:
                # Convert index back to node index.
                node_index = manager.IndexToNode(index)
                # Accumulate prize for visited nodes.
                total_prize += prizes[node_index]
    # Return the total prize collected.
    return total_prize