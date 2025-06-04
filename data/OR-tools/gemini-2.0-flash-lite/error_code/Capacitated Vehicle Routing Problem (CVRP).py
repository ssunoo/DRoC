
# Capacitated Vehicle Routing Problem (CVRP)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    # The code solves the Capacitated Vehicle Routing Problem (CVRP) using the OR-Tools library.
    # It takes the distance matrix, number of vehicles, depot, demands, and vehicle capacity as input.
    # It calculates the objective value (total distance) of the solution.

    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    def solve(distance_matrix, num_vehicle, depot, demands, vehicle_capacity):
        """Solves the CVRP and returns the objective value."""

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(distance_matrix), num_vehicle, depot
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

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
            demand_callback_index, 0,  # null capacity slack
            vehicle_capacity,  # vehicle maximum capacities
            True,  # start cumul to zero
            "Capacity",
        )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy =
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        obj = float('inf')
        if solution:
            obj = solution.ObjectiveValue()
        return obj
