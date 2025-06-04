
# Vehicle Routing Problem with Service Time and Duration Limit (VRPSL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys

    obj = -1

    # Create the routing index manager.
    # This manages the indices used by the routing solver.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model.
    # This is the main object that solves the routing problem.
    routing = pywrapcp.RoutingModel(manager)

    # Create a mutable copy of service_time and ensure service time at depot is 0.
    # Service time is typically for customer nodes, not the depot itself.
    service_time_at_nodes = list(service_time)
    if depot < len(service_time_at_nodes): # Ensure depot index is valid
        service_time_at_nodes[depot] = 0

    # Create and register a transit callback that includes travel time and service time.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes plus service time at the from_node."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # The time spent at 'from_node' (service_time) is incurred before traveling to 'to_node'.
        return time_matrix[from_node][to_node] + service_time_at_nodes[from_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    # The objective is to minimize the total time (travel + service).
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    # This dimension will track the cumulative time for each vehicle.
    # The 'capacity' argument enforces the duration_limit for each route.
    # 'slack_max' allows for waiting time if a vehicle arrives early.
    # 'fix_start_cumul_to_zero' means vehicles start at time 0 from the depot.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        sys.maxsize,  # slack_max: maximum waiting time at a node (set to a very large value)
        duration_limit,  # capacity: maximum time per vehicle (duration limit)
        True,            # fix_start_cumul_to_zero: vehicles start at time 0
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(5) # Add a reasonable time limit for solver

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        obj = -1 # No solution found

    return obj
