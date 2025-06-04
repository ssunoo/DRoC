# Vehicle Routing Problem with Duration Limit and Open Start and End locations (OVRPL)
# [('user', 'The solution failed the code execution test: Error: \'bool\' object has no attribute \'SetGlobalSpanCostCoefficient\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmp5j6bzpjo.py", line 78, in <module>\n    result = solve(time_matrix, num_vehicle, duration_limit)\n  File "/tmp/tmp5j6bzpjo.py", line 46, in solve\n    time_dimension.SetGlobalSpanCostCoefficient(100)\nAttributeError: \'bool\' object has no attribute \'SetGlobalSpanCostCoefficient\'\n\n')]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, num_vehicle: int, duration_limit: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        num_vehicle: the number of the vehicle
        duration_limit: the time duration of each route is upper bounded by the duration limit 

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {}
    data['distance_matrix'] = time_matrix
    data['num_vehicles'] = num_vehicle
    data['starts'] = [0] * num_vehicle  # Assuming all vehicles start at node 0
    data['ends'] = [0] * num_vehicle    # Assuming all vehicles end at node 0

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Duration dimension with upper bound
    time_dimension = routing.AddDimension(
        transit_callback_index,
        0,  # slack
        duration_limit,  # maximum travel time per vehicle
        True,  # start cumul to zero
        'Time')

    # Set global span cost coefficient to minimize the maximum route duration
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Add capacity dimension to enforce maximum route duration
    # (This is already done via AddDimension with upper bound, but ensure it's correctly configured)
    # No additional capacity dimension needed if duration is handled via AddDimension

    # Set search parameters with a time limit
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 30  # Set time limit for search

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if solution exists
    if solution:
        # The objective is the total travel time
        return solution.ObjectiveValue()
    else:
        return -1