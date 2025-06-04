
# Vehicle Routing Problem with Time Windows and Multiple Depots and Duration Limit (VRPTWMDL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicles: int, duration_limit: int):
        """Solves the Vehicle Routing Problem with Time Windows and Multiple Depots and Duration Limit (VRPTWMDL) using OR-Tools.

        Args:
            time_matrix: A list of lists representing the time taken to travel between locations.
            time_windows: A list of tuples representing the time windows for each node.
            starts: A list of starting depot indices for each vehicle.
            ends: A list of ending depot indices for each vehicle.
            num_vehicles: The number of vehicles available.
            duration_limit: The maximum allowed duration for a route.

        Returns:
            The objective value of the solution.
        """
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(time_matrix),
            num_vehicles,
            starts,
            ends
        )

        # Create the routing model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def time_callback(from_index, to_index):
            """Returns the travel time between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return time_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(time_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Time Windows constraint.
        time = "Time"
        routing.AddDimension(
            transit_callback_index,
            duration_limit,
            duration_limit,
            False,  # start cumul to zero
            time
        )
        time_dimension = routing.GetDimensionOrDie(time)
        # Add time window constraints for each location.
        for location_idx, time_window in enumerate(time_windows):
            if location_idx == manager.StartOrEndIndex(0):  # Skip depot
                continue
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # Add time window constraints for each vehicle start node.
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(time_windows[0][0], time_windows[0][1])  # Assuming depot time window
            # Add time window constraints for each vehicle end node.
            index = routing.End(vehicle_id)
            time_dimension.CumulVar(index).SetRange(time_windows[0][0], time_windows[0][1])  # Assuming depot time window

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Return the objective value.
        if solution:
            obj = solution.ObjectiveValue()
        return obj
