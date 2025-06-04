
# Vehicle Routing with Pickups and Deliveries and Distance Limit (PDPL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

    obj = -1

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, depot)

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

    # Define cost of each arc. This is typically used for total distance minimization.
    # However, for PDPL with distance limit, the objective is often to minimize the longest route.
    # The SetGlobalSpanCostCoefficient below will make the longest route the predominant factor.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance Dimension to track cumulative distance and enforce limits.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        distance_limit,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # Set a large coefficient for the global span of the routes to minimize the length of the longest route.
    # As per the context, this makes the global span the predominant factor in the objective function.
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define pickup and delivery requests and add constraints.
    for request in pickups_deliveries:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        # AddPickupAndDelivery implicitly ensures same vehicle and pickup before delivery.
        routing.AddPickupAndDelivery(pickup_index, delivery_index)

    # Set search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(30) # Set a time limit for the solver.

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value from the solution.
    if solution:
        obj = solution.ObjectiveValue()

    return obj
