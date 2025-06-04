# Vehicle Routing Problem with Service Time and Duration Limit (VRPSL)
# [('user', 'The solution failed the code execution test: Error: \'bool\' object has no attribute \'CumulVar\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpjyf1ygg8.py", line 95, in <module>\n    result = solve(time_matrix, num_vehicle, depot, service_time, duration_limit)\n  File "/tmp/tmpjyf1ygg8.py", line 63, in solve\n    time_dimension.CumulVar(end_index) <= duration_limit\nAttributeError: \'bool\' object has no attribute \'CumulVar\'\n\n')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        service_time: service time for each customer node
        duration_limit: the time duration of each route is upper bounded by the duration limit 

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

    # Set the cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    time_dimension = routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        duration_limit,  # maximum travel time per vehicle
        True,  # force start cumul to zero
        'Time')

    # Add service time as a separate dimension.
    def service_time_callback(from_index):
        node = manager.IndexToNode(from_index)
        return service_time[node]

    service_time_callback_index = routing.RegisterUnaryTransitCallback(service_time_callback)

    # Add a dimension for service time.
    routing.AddDimension(
        service_time_callback_index,
        0,  # no slack
        duration_limit,  # maximum total service time
        True,  # force start cumul to zero
        'ServiceTime')

    # Set maximum route duration.
    solver = routing.solver()
    for vehicle_id in range(num_vehicle):
        start_index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        # Add constraint for route duration.
        # Corrected the error by explicitly getting the CumulVar for the end node of each route.
        solver.Add(
            time_dimension.CumulVar(end_index) <= duration_limit
        )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Optional: set a time limit for the search.
    # search_parameters.time_limit.seconds = 30

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1