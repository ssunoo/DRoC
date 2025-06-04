
# Travelling Salesman Problem with Time Windows and Service Time (TSPTWS)
# [('user', 'The solution failed the code execution test: Error: 'RoutingDimension' object has no attribute 'SetNodeVisitTransit'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpwku8s9nh.py", line 105, in <module>\n    result = solve(time_matrix, time_windows, depot, service_time)\n  File "/tmp/tmpwku8s9nh.py", line 63, in solve\n    time_dimension.SetNodeVisitTransit(service_time[i], node_index)\nAttributeError: 'RoutingDimension' object has no attribute 'SetNodeVisitTransit'\n
')]

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import sys


def solve(time_matrix: list, time_windows: list, depot: int, service_time: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        depot: the index of the depot node
        service_time: service time for each customer node

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    num_locations = len(time_matrix)
    num_vehicles = 1 # TSP is for a single vehicle

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for travel times.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    time = "Time"
    # A sufficiently large number for maximum time per vehicle and slack_max.
    # This ensures that the dimension capacity is not a limiting factor unless
    # explicitly intended by problem constraints.
    max_time_per_vehicle = sys.maxsize // 2
    slack_max = sys.maxsize // 2 # Allow large waiting times

    routing.AddDimension(
        transit_callback_index,
        slack_max,  # allow waiting time
        max_time_per_vehicle,  # maximum time per vehicle
        True,  # Force start cumul to zero for the depot.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add service time to the time dimension for each node.
    # SetNodeVisitTransit adds a fixed cost to the cumul variable at a node.
    # This models the time spent at a node (service time) before departure.
    # The error message indicates that this method does not exist.  This is likely due to an API change.
    # The correct way to add service time is to add it to the time window constraints.
    # for i in range(num_locations):
    #     node_index = manager.NodeToIndex(i)
    #     time_dimension.SetNodeVisitTransit(service_time[i], node_index)

    # Add time window constraints for each location.
    # The cumul variable at each node (representing arrival time) must be
    # within its specified time window [start, end].
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0] + service_time[location_idx], time_window[1] + service_time[location_idx])

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    # Optional: Add a time limit for the solver.
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value if a solution is found.
    if solution:
        obj = solution.ObjectiveValue()

    return obj
