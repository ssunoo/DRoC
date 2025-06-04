
# Capacitated Vehicle Routing Problem with Time Windows, Multiple Depots, and Time Limit (CVRPTWMDL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys

    num_locations = len(time_matrix)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicle, starts, ends)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback for time.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    capacity_name = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        vehicle_capacities,
        True,  # start cumul to zero
        capacity_name,
    )

    # Add Time Windows constraint.
    time_name = "Time"
    # The slack_max should be large enough to allow waiting.
    # The capacity should be the maximum allowed time for a route (duration_limit).
    routing.AddDimension(
        transit_callback_index,
        sys.maxsize,  # allow waiting time (large slack)
        duration_limit,  # maximum time per vehicle (route duration limit)
        False,  # Don't force start cumul to zero.
        time_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_name)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle's start and end nodes.
    # This is crucial for multiple depots.
    for vehicle_id in range(num_vehicle):
        # Set time window for the vehicle's start node (depot)
        start_node_idx = starts[vehicle_id]
        time_dimension.CumulVar(routing.Start(vehicle_id)).SetRange(
            time_windows[start_node_idx][0],
            time_windows[start_node_idx][1]
        )
        # Set time window for the vehicle's end node (depot)
        end_node_idx = ends[vehicle_id]
        time_dimension.CumulVar(routing.End(vehicle_id)).SetRange(
            time_windows[end_node_idx][0],
            time_windows[end_node_idx][1]
        )

    # Instantiate route start and end times to produce feasible times.
    # This helps the solver to minimize the total time.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Add a cost to minimize the maximum route duration (makespan).
    # This can help guide the solver to find solutions faster for time-constrained problems.
    # The coefficient (e.g., 100) can be tuned based on problem scale.
    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )

    # Set time limit.
    search_parameters.time_limit.seconds = 30

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    obj = -1 # Default value if no solution is found
    if solution:
        obj = solution.ObjectiveValue()

    return obj
