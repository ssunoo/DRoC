# Prize Collecting Vehicle Routing Problem with Time Windows and Multiple Depots (PCVRPTWMD)
# [('user', 'The solution failed the code execution test: Error: \'bool\' object has no attribute \'CumulVar\'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmprq6pj50z.py", line 112, in <module>\n    result = solve(time_matrix, time_windows, starts, ends, num_vehicle, prizes, max_time)\n  File "/tmp/tmprq6pj50z.py", line 33, in solve\n    time_dimension.CumulVar(index).SetRange(window[0], window[1])\nAttributeError: \'bool\' object has no attribute \'CumulVar\'\n\n')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicle: int, prizes: list, max_time: int):
    # Create the routing index manager.
    num_locations = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicle, starts, ends)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension.
    time_dimension = routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_time,  # maximum travel time
        False,  # don't force start cumul to zero
        'Time')

    # Set time window constraints for each location.
    for location_idx, window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(window[0], window[1])

    # Set time window constraints for each vehicle start node.
    for vehicle_id in range(num_vehicle):
        start_index = routing.Start(vehicle_id)
        depot_idx = starts[vehicle_id]
        depot_window = time_windows[depot_idx]
        time_dimension.CumulVar(start_index).SetRange(depot_window[0], depot_window[1])

    # Add prize collection as a dimension to maximize total prize.
    # Since OR-Tools does not directly support prize collection, we model it as a reward.
    # We can add a dummy dimension to accumulate prizes.
    def prize_callback(from_index):
        node = manager.IndexToNode(from_index)
        return prizes[node]

    prize_callback_index = routing.RegisterUnaryTransitCallback(prize_callback)

    # Add a dimension for prizes to maximize.
    routing.AddDimensionWithVehicleTransitCallback(
        prize_callback_index,
        0,  # no slack
        sum(prizes),  # maximum total prize
        True,  # start cumul to zero
        'Prize')

    prize_dimension = routing.GetDimensionOrDie('Prize')

    # Set objective to maximize prize collected.
    # OR-Tools minimizes by default, so we set a negative weight.
    routing.SetObjective(prize_dimension.CumulVar(0))

    # Add maximum route duration constraint.
    for vehicle_id in range(num_vehicle):
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            max_time,  # maximum route duration
            True,  # start cumul to zero
            f'Duration_{vehicle_id}')
        duration_var = routing.GetDimensionOrDie(f'Duration_{vehicle_id}').CumulVar(routing.End(vehicle_id))
        routing.AddConstraint(duration_var <= max_time)

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Compute total prize collected.
    total_prize = 0
    if solution:
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                total_prize += prizes[node]
                index = solution.Value(routing.NextVar(index))
    return total_prize if solution else -1
