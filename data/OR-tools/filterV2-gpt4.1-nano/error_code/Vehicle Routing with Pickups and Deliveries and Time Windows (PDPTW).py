# Vehicle Routing with Pickups and Deliveries and Time Windows (PDPTW)
# [('user', 'The solution failed the code execution test: Error: CP Solver fail\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpf8y0v34a.py", line 101, in <module>\n    result = solve(time_matrix, time_windows, pickups_deliveries, num_vehicle, depot)\n  File "/tmp/tmpf8y0v34a.py", line 43, in solve\n    time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])\n  File "/disks/local/r.2t_6/r13922049/miniconda3/envs/AI/lib/python3.10/site-packages/ortools/constraint_solver/pywrapcp.py", line 2620, in SetRange\n    return _pywrapcp.IntExpr_SetRange(self, l, u)\nException: CP Solver fail\n\n')] # Import necessary modules from OR-Tools for routing and constraint solving
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int):
    # Create the routing index manager to manage the conversion between node indices and internal routing indices.
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Instantiate the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Define a callback function to return travel times between nodes.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    # Register the callback with the routing model.
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc to be the travel time.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to model time with waiting times and maximum route duration.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time at nodes
        30,  # maximum time per vehicle route
        False,  # Do not force start cumul to zero.
        time,
    )
    # Retrieve the time dimension to add constraints.
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location, including depot.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        # Apply constraints only if the node is part of the route.
        if location_idx == depot:
            # Optionally, set depot time window if provided.
            if time_window:
                time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        else:
            # For other nodes, set the time window.
            if time_window:
                time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        # Set range for start node if depot has a time window.
        if depot is not None and index == manager.NodeToIndex(depot):
            depot_time_window = time_windows[depot]
            if depot_time_window:
                time_dimension.CumulVar(index).SetRange(
                    depot_time_window[0], depot_time_window[1]
                )

    # Add Pickup and Delivery constraints.
    for pickup_node, delivery_node in pickups_deliveries:
        pickup_index = manager.NodeToIndex(pickup_node)
        delivery_index = manager.NodeToIndex(delivery_node)
        # Link pickup and delivery nodes.
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure that pickup and delivery are assigned to the same vehicle.
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        # Enforce that pickup occurs before delivery.
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Minimize the total time for all routes.
    for i in range(num_vehicle):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Set search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the routing problem with the specified parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1