
    def solve(time_matrix: list, num_vehicle: int, duration_limit: int):  # Corrected function signature
        """
        Args:
            time_matrix: contains the integer travel times between locations
           num_vehicle: the number of the vehicle
           duration_limit: the time duration of each route is upper bounded by the duration limit

        Returns:
            obj: a number representing the objective value of the solution
        """
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(time_matrix),
            num_vehicle,
            0,  # Define depot as the start and end node
            0
        )

        # Create Routing Model.
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

        # Add Time constraint.
        dimension_name = "Time"
        routing.AddDimension(
            transit_callback_index,
            0,  # allow waiting time
            duration_limit,  # maximum time per vehicle
            False,  # start cumul to zero
            dimension_name,
        )
        time_dimension = routing.GetDimensionOrDie(dimension_name)
        time_dimension.SetGlobalSpanCostCoefficient(100)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.time_limit.seconds = 10  # Set a time limit

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        if solution:
            return solution.ObjectiveValue()
        else:
            return -1
