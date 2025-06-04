
# Travelling Salesman Problem with Time Windows (TSPTW)
# [('user', 'The generated code cannot run or time out.')]

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


    def solve(time_matrix, time_windows, depot):
        """Solves the TSPTW problem and returns the objective value."""
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(time_matrix), 1, depot
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def time_callback(from_index, to_index):
            """Returns the travel time between the two nodes."""
            # Convert from routing variable Index to time matrix NodeIndex.
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
            30,  # allow waiting time
            30,  # maximum time per vehicle
            False,  # Don't force start cumul to zero.
            time,
        )
        time_dimension = routing.GetDimensionOrDie(time)
        # Add time window constraints for each location except depot.
        for location_idx, time_window in enumerate(time_windows):
            if location_idx == depot:
                continue
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # Add time window constraints for each vehicle start node.
        depot_idx = depot
        for vehicle_id in range(1):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                time_windows[depot_idx][0], time_windows[depot_idx][1]
            )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy =
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

        # Solve the problem.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        obj = -1
        if solution:
            obj = solution.ObjectiveValue()
        return obj
