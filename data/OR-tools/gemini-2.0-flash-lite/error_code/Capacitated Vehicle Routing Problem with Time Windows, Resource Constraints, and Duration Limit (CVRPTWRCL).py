
# Capacitated Vehicle Routing Problem with Time Windows, Resource Constraints, and Duration Limit (CVRPTWRCL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def solve(data):
    """Solves the CVRPTWRCL problem."""
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["locations"]),
        data["num_vehicles"],
        data["depot"],
    )

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )

    # Add Time Windows constraint.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(
        time_callback_index,
        data["time_window_max"],  # allow waiting time
        data["time_window_max"],  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        "Time",
    )
    time_dimension = routing.GetDimensionOrDie("Time")
    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for the depot.
    depot_idx = data["depot"]
    index = manager.NodeToIndex(depot_idx)
    time_dimension.CumulVar(index).SetRange(data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1])

    # Add time window constraints for the start and end of each route.
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.SetSpanCostCoefficientForVehicle(100, vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1])
        index = routing.End(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1])

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.seconds = 10

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    return solution
