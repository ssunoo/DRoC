
# Vehicle Routing with Pickups and Deliveries, Service Time, and Duration Limit (PDPSL)
# [('user', 'The obj. is far from the optimum, and you may not have considered all the constraints.')]

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve(time_matrix: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    obj = -1

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define a callback for transit time (travel time + service time).
    def time_callback(from_index, to_index):
        # Convert from routing index to node index.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # The time taken to travel from 'from_node' to 'to_node' plus service time at 'from_node'.
        return time_matrix[from_node][to_node] + service_time[from_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension.
    # The 'slack_max' is 0, meaning no waiting time is allowed at nodes.
    # The 'capacity' is the duration_limit for each vehicle.
    # 'fix_start_cumul_to_zero' is True, meaning the time starts at 0 for each vehicle.
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack (max waiting time at node)
        duration_limit,  # vehicle maximum travel time (capacity)
        True,  # start cumul to zero
        "Time"
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    # Add pickup and delivery requests.
    for request in pickups_deliveries:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure pickup occurs before delivery in terms of time.
        # This is often handled by AddPickupAndDelivery with a time dimension,
        # but explicitly adding it ensures the constraint.
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(30) # Set a reasonable time limit for solving

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        print("No solution found!")

    return obj
