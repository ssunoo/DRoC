# Vehicle Routing Problem with Service Time (VRPS)
# [('user', 'The solution failed the code execution test: Error: \'RoutingModel\' object has no attribute \'AddBreak\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmp5prvua26.py", line 92, in <module>\n    result = solve(time_matrix, num_vehicle, depot, service_time)\n  File "/tmp/tmp5prvua26.py", line 64, in solve\n    routing.AddBreak(break_interval, v)\nAttributeError: \'RoutingModel\' object has no attribute \'AddBreak\'
\n')]
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
    # Instantiate the data model.
    data = {}
    data['time_matrix'] = time_matrix
    data['num_vehicles'] = num_vehicle
    data['depot'] = depot
    data['service_time'] = service_time

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes, including service time at the from_node."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node] + data['service_time'][from_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        100000,  # large max travel time
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Add service time as break intervals for each vehicle.
    for v in range(data['num_vehicles']):
        start_var = time_dimension.CumulVar(routing.Start(v))
        # Set start time for each vehicle.
        start_time = (v + 1) * 15  # example start times, can be parameterized
        start_var.SetValue(start_time)
        # Define break start time window: between 25 and 45 minutes after route start.
        break_start_min = start_time + 25
        break_start_max = start_time + 45
        # Create break interval as an interval variable.
        break_start_var = routing.solver().IntVar(break_start_min, break_start_max, f'break_start_v{v}')
        break_duration = 5
        # Create break interval using FixedDurationIntervalVar.
        break_interval = routing.solver().FixedDurationIntervalVar(break_start_var, break_duration, f'break_v{v}')
        # Add break to the route.
        routing.AddBreak(break_interval, v)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1