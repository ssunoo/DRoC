# Prize Collecting Vehicle Routing Problem (PCVRP)
# This implementation aims to maximize the collected prizes while respecting the maximum distance constraint for each vehicle.
# The problem involves routing multiple vehicles to visit nodes, collecting prizes, and minimizing total travel distance.

# Import necessary modules from OR-Tools for constraint programming.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, prizes: list, max_distance: int, num_vehicle: int, depot: int):
    # Create the routing index manager.
    # This manages the conversion between the node indices and the internal routing model indices.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

    # Create routing model.
    # This is the main data structure for solving the VRP.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    # This callback returns the distance between two nodes.
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the cost of each arc.
    # This tells the solver to minimize the total distance traveled.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance dimension to enforce max distance constraint.
    # This dimension keeps track of the total distance traveled by each vehicle.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,  # maximum distance per vehicle
        True,  # start cumul to zero
        'Distance')

    # Add disjunctions for prize collection (drop nodes with penalty).
    # This allows the solver to skip certain nodes at a penalty (prize value).
    for node in range(1, len(distance_matrix)):
        routing.AddDisjunction([manager.NodeToIndex(node)], prizes[node])

    # Setting first solution heuristic.
    # Defines the strategy for finding an initial feasible solution.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(15)  # Limit search time to 15 seconds

    # Solve the problem.
    # This attempts to find the best route configuration.
    assignment = routing.SolveWithParameters(search_parameters)

    # Calculate total prize collected.
    total_prize = 0
    if assignment:
        # For each node, check if it is visited.
        for node in range(1, len(distance_matrix)):
            index = manager.NodeToIndex(node)
            # If the node is visited in the route.
            if assignment.Value(routing.NextVar(index)) != index:
                total_prize += prizes[node]
        return total_prize
    else:
        # No feasible solution found.
        return 0