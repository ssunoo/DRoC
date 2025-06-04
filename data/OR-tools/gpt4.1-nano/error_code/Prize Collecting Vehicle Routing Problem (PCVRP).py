# Prize Collecting Vehicle Routing Problem (PCVRP)
# This code solves a variant of the vehicle routing problem where the goal is to maximize the collected prizes within a maximum distance constraint.
# Import necessary modules from OR-Tools.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, prizes: list, max_distance: int, num_vehicle: int, depot: int):
    # Create the routing index manager, which manages the conversion between node indices and routing indices.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes.
    def distance_callback(from_index, to_index):
        # Convert routing indices to node indices.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the distance between the nodes from the distance matrix.
        return distance_matrix[from_node][to_node]

    # Register the transit callback and get its index.
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc (edge) to be the distance as evaluated by the callback.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to enforce a maximum travel distance constraint for each vehicle.
    routing.AddDimension(
        transit_callback_index,  # Transit callback index.
        0,  # No slack time.
        max_distance,  # Maximum travel distance per vehicle.
        True,  # Start cumul to zero.
        'Distance')  # Dimension name.

    # Retrieve the distance dimension to access its properties.
    distance_dimension = routing.GetDimensionOrDie('Distance')

    # Add disjunctions with penalties to incentivize visiting nodes with higher prizes.
    for node in range(1, len(distance_matrix)):
        # Convert node to routing index.
        index = manager.NodeToIndex(node)
        # Penalty is negative prize to incentivize visiting high prize nodes.
        penalty = -prizes[node]
        # Add disjunction with penalty for skipping the node.
        routing.AddDisjunction([index], penalty)

    # Set search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use a heuristic to find an initial solution.
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Use guided local search for improvement.
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # Set a time limit for the solver.
    search_parameters.time_limit.FromSeconds(15)

    # Solve the routing problem with the specified parameters.
    assignment = routing.SolveWithParameters(search_parameters)

    # Initialize total prize collected.
    total_prize = 0
    # If a solution is found, compute the total prize collected.
    if assignment:
        # Iterate over each vehicle.
        for v in range(num_vehicle):
            # Start from the start node of the vehicle.
            index = routing.Start(v)
            # Traverse the route until the end.
            while not routing.IsEnd(index):
                # Convert routing index to node index.
                node = manager.IndexToNode(index)
                # Add the prize of the current node.
                total_prize += prizes[node]
                # Move to the next node in the route.
                index = assignment.Value(routing.NextVar(index))
    # Return the total prize collected.
    return total_prize