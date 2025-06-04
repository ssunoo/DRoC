
# Capacitated Vehicle Routing Problem with Time Windows and Open Start and End locations (OCVRPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    num_locations = len(time_matrix)
    depot = 0  # Assuming the first location (index 0) is the depot for starts

    # Create the routing index manager.
    # For open start/end, if all vehicles start at the same depot and can end anywhere,
    # initializing with a single depot is common, then setting end arc costs to zero.
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicle, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for time.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc. The objective is to minimize total time.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    capacity_dimension_name = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,
        True,  # start cumul to zero
        capacity_dimension_name,
    )

    # Add Time Windows constraint.
    time_dimension_name = "Time"
    # Use a sufficiently large constant for max_time_per_vehicle and slack_max.
    # Reduced from 10^9/10^6 to 10000 for better performance and to avoid excessively large search space.
    max_time_value = 10000  # A sufficiently large but more constrained number

    routing.AddDimension(
        transit_callback_index,
        max_time_value,  # allow waiting time (slack_max)
        max_time_value,  # maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero.
        time_dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        # The depot's time window is handled by the vehicle start nodes below.
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot).
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )

    # Open End locations: Vehicles are not required to return to the depot.
    # Set the cost of the final arc (from the last visited customer to the depot) to zero.
    # This effectively means there's no penalty for not returning to the depot.
    for vehicle_id in range(num_vehicle):
        routing.SetEndArcCostForVehicle(manager.NodeToIndex(depot), vehicle_id, 0)

    # Instantiate route start and end times to produce feasible times and minimize total time.
    # Minimizing start time is generally good.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        # For open end locations, minimizing the time at the conceptual end depot might not be desired
        # or could make the problem harder to solve. The primary objective is already minimizing total time.
        # Removed: routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    # Optional: Add a metaheuristic for better solutions on larger problems.
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    # Optional: Set a time limit for the solver.
    search_parameters.time_limit.FromSeconds(10)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    obj = -1
    if solution:
        obj = solution.ObjectiveValue()
    return obj
