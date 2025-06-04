
# Vehicle Routing with Pickups and Deliveries, Time Windows, Service Time, and Duration Limit (PDPTWSL)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

    obj = -1

    # Create the data model.
    data = {}
    data["time_matrix"] = time_matrix  # Travel times between locations
    data["time_windows"] = time_windows  # Time window for each location (start, end)
    data["pickups_deliveries"] = pickups_deliveries  # Pairs of pickup and delivery locations
    data["num_vehicles"] = num_vehicle  # Number of vehicles
    data["depot"] = depot  # Depot location index
    data["service_time"] = service_time  # Service time at each location
    data["duration_limit"] = duration_limit  # Maximum route duration

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["time_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc (travel time).
    def travel_time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(travel_time_callback)

    # Define cost of travel for each vehicle (minimize total travel time).
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension with time windows, service times, and duration limits.
    time_dimension_name = "Time"
    routing.AddDimension(
        transit_callback_index,
        data["duration_limit"],  # slack_max: allow waiting time up to duration_limit
        data["duration_limit"],  # capacity: maximum total time for a route
        True,  # fix_start_cumul_to_zero: start cumul from zero at depot
        time_dimension_name,
    )
    time_dimension = routing.GetDimensionOrDie(time_dimension_name)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(location_idx)
        if location_idx == data["depot"]:
            # Apply depot time window to start and end nodes of all vehicles
            for vehicle_id in range(data["num_vehicles"]):
                # Start node of vehicle
                time_dimension.SetCumulVarRange(
                    routing.Start(vehicle_id), time_window[0], time_window[1]
                )
                # End node of vehicle
                time_dimension.SetCumulVarRange(
                    routing.End(vehicle_id), time_window[0], time_window[1]
                )
        else:
            time_dimension.SetCumulVarRange(index, time_window[0], time_window[1])

    # Add service time at each node.
    for i in range(len(data["time_matrix"])):
        index = manager.NodeToIndex(i)
        time_dimension.SetNodeServiceTime(index, data["service_time"][i])

    # Add duration limit for each vehicle route.
    for vehicle_id in range(data["num_vehicles"]):
        routing.solver().Add(
            time_dimension.CumulVar(routing.End(vehicle_id)) <= data["duration_limit"]
        )

    # Add Pickup and Delivery constraints.
    for request in data["pickups_deliveries"]:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure delivery happens after pickup and by the same vehicle.
        # The AddPickupAndDelivery method already handles the same vehicle constraint.
        # We explicitly add the time constraint: delivery time >= pickup time + service time at pickup.
        routing.solver().Add(
            time_dimension.CumulVar(delivery_index)
            >= time_dimension.CumulVar(pickup_index)
            + time_dimension.GetNodeServiceTime(pickup_index)
        )

    # Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(30)  # Set a time limit for the search

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()

    return obj
