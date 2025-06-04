# Vehicle Routing Problem with Service Time (VRPS)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, num_vehicle: int, depot: int, service_time: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        service_time: service time for each customer node 

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        10,  # allow waiting time
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Set the start time for each vehicle.
    for v in range(num_vehicle):
        index = routing.Start(v)
        time_dimension.CumulVar(index).SetValue(0)

    # Add service time as constraints.
    for node in range(len(time_matrix)):
        index = manager.NodeToIndex(node)
        if node != depot:
            # Add a constraint that the cumul variable at the node is at least the service time.
            # This models the service time delay.
            time_dimension.CumulVar(index).SetValue(service_time[node])

    # Add break intervals for each vehicle.
    for v in range(num_vehicle):
        start_var = time_dimension.CumulVar(routing.Start(v))
        # Break start time between 25 and 45 minutes after route start.
        break_start = routing.solver().Sum([
            routing.solver().IntVar(25, 45), start_var])
        # Create break interval variable.
        break_interval = routing.solver().FixedDurationIntervalVar(break_start, 5, f'Break_{v}')
        # Assign break interval to vehicle.
        time_dimension.SetBreakIntervalsOfVehicle([break_interval], v, [] )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.FromSeconds(10)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    obj = -1
    if solution:
        obj = solution.ObjectiveValue()
    return obj
