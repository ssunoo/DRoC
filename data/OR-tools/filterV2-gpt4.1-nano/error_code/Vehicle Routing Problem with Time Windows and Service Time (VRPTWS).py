# Vehicle Routing Problem with Time Windows and Service Time (VRPTWS)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, service_time: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        num_vehicle: the number of the vehicle
        depot: the index of the depot node
        service_time: service time for each customer node

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'num_vehicles': num_vehicle,
        'depot': depot,
        'service_time': service_time
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback that includes service time
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes, including service time at the from_node."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        travel_time = data['time_matrix'][from_node][to_node]
        service = data['service_time'][from_node]
        return travel_time + service

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension with slack for waiting
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        180,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for all nodes, including depot
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        start_min, end_max = time_window
        time_dimension.CumulVar(index).SetRange(start_min, end_max)

    # Set start node time window constraints for each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Set first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    # Optional: set a time limit for the solver to prevent timeouts
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if solution is feasible
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1