# Prize Collecting Vehicle Routing Problem with Time Windows (PCVRPTW)
# This implementation uses Google OR-Tools to solve a VRP with time windows and prize collection.
# The objective is to maximize the total prize collected while respecting time windows and maximum route duration.
# Import necessary modules from OR-Tools.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, prizes: list, max_duration: int):
    # Create the routing index manager.
    # Manages the conversion between node indices and internal routing indices.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create the routing model.
    # Represents the VRP problem instance.
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times.
    # Defines the travel time between nodes.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    # Register the transit callback.
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc.
    # Uses travel time as the arc cost.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension.
    # Enforces time window constraints and allows waiting.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        max_duration,  # maximum route duration
        False,  # Don't force start cumul to zero.
        time
    )
    # Retrieve the time dimension for further constraints.
    time_dimension = routing.GetDimensionOrDie(time)

    # Validate and set time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        start, end = time_window
        # Ensure the time window is within feasible bounds.
        if start < 0:
            start = 0
        if end > max_duration:
            end = max_duration
        index = manager.NodeToIndex(location_idx)
        try:
            # Set the time window for the node.
            time_dimension.CumulVar(index).SetRange(start, end)
        except Exception:
            # If setting range fails, the problem might be infeasible.
            # Handle accordingly, e.g., skip or set to feasible bounds.
            time_dimension.CumulVar(index).SetRange(0, max_duration)

    # Set time window constraints for each vehicle start node.
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        depot_time_window = time_windows[depot]
        start, end = depot_time_window
        if start < 0:
            start = 0
        if end > max_duration:
            end = max_duration
        try:
            # Set the time window for the start node of each vehicle.
            time_dimension.CumulVar(index).SetRange(start, end)
        except Exception:
            # Handle potential exceptions.
            time_dimension.CumulVar(index).SetRange(0, max_duration)

    # Add prize collection as disjunctions with penalty.
    # Allows skipping nodes at a penalty cost.
    for node in range(1, len(prizes)):
        routing.AddDisjunction([manager.NodeToIndex(node)], prizes[node])

    # Add maximum route duration constraint.
    # Ensures each route does not exceed max_duration.
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        # Minimize the start node's cumulative time.
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index))
        index_end = routing.End(vehicle_id)
        # Minimize the end node's cumulative time.
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index_end))
        # Add constraint for route duration.
        routing.solver().Add(
            time_dimension.CumulVar(index_end) <= max_duration
        )

    # Set search parameters.
    # Defines the strategy for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(15)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Compute total prize collected.
    total_prize = 0
    if assignment:
        # Iterate over each vehicle to calculate total prize.
        for v in range(num_vehicle):
            index = routing.Start(v)
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                total_prize += prizes[node]
                index = assignment.Value(routing.NextVar(index))
        return total_prize
    else:
        # No feasible solution found.
        return 0