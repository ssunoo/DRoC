
# Capacitated Vehicle Routing Problem with Time Windows (CVRPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int, depot: int):
        """
        Args:
            time_matrix: contains the integer travel times between locations
           time_windows: the list of tuples for time windows of the customers
        demands: the list of integer customer demands
       vehicle_capacities: the capacity of each vehicle
         num_vehicle: the number of the vehicle
            depot: the index of the depot node

        Returns:
            obj: a number representing the objective value of the solution
        """
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(time_matrix), num_vehicle, depot
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

        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = manager.IndexToNode(from_index)
            return demands[from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            vehicle_capacities,  # vehicle maximum capacities
            True,  # start cumul to zero
            "Capacity",
        )

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
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # Add time window constraints for each vehicle start node.  The depot time window should be applied here.
        for vehicle_id in range(num_vehicle):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                time_windows[depot][0], time_windows[depot][1]
            )

        # Instantiate route start and end times to produce feasible times.
        for i in range(num_vehicle):
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(i))
            )
            routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Return the objective value of the solution
        if solution:
            return solution.ObjectiveValue()
        else:
            return -1
