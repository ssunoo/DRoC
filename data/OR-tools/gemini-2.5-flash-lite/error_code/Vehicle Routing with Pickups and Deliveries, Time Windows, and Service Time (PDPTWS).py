
# Vehicle Routing with Pickups and Deliveries, Time Windows, and Service Time (PDPTWS)
# [('user', 'The solution failed the code execution test: Error: 'bool' object has no attribute 'SetCumulVarRange'\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmpz0cqmb86.py", line 114, in <module>\n    result = solve(time_matrix, time_windows, pickups_deliveries, num_vehicle, depot, service_time)\n  File "/tmp/tmpz0cqmb86.py", line 55, in solve\n    time_dimension.SetCumulVarRange(index, time_window[0], time_window[1])\nAttributeError: 'bool' object has no attribute 'SetCumulVarRange'\n
')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list):
    """
    Args:
        time_matrix: contains the integer travel times between locations
      time_windows: the list of tuples for time windows of the customers
     pickups_deliveries: a list of pairs of pickup and delivery locations
        num_vehicle: the number of the vehicle
      depot: the index of the depot node
           service_time: service time for each customer node

    Returns:
        obj: a number representing the objective value of the solution
    """
    obj = -1

    # 1. Create the routing index manager.
    num_locations = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicle, depot)

    # 2. Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # 3. Define cost of each arc (travel time + service time at 'from' node).
    def transit_callback(from_index, to_index):
        """Returns the travel time between the two nodes plus service time at the 'from' node."""
        # Convert from routing variable Index to node Index.
        from_node = manager.IndexToNode(from_index)
        # The cost includes travel time from from_node to to_node
        # and service time at the from_node.
        return time_matrix[from_node][manager.IndexToNode(to_index)] + service_time[from_node]

    transit_callback_index = routing.RegisterTransitCallback(transit_callback)

    # Define cost of travel for each vehicle.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # 4. Add Time Windows constraint.
    # The maximum value for a cumul variable (time). This should be large enough
    # to cover any possible route duration. A large constant is used here.
    max_time_per_vehicle = 1000000 # A sufficiently large number

    time_dimension = routing.AddDimension(
        transit_callback_index,
        max_time_per_vehicle,  # allow waiting time at nodes (slack_max)
        max_time_per_vehicle,  # maximum time per vehicle (capacity)
        False,                 # Don't force start cumul to zero; time windows handle start times.
        "Time")

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        # The error occurs here, check if time_window is valid before using it.
        if isinstance(time_window, tuple) and len(time_window) == 2: # Check if time_window is a tuple of length 2
            time_dimension.SetCumulVarRange(index, time_window[0], time_window[1])
        else:
            print(f"Warning: Invalid time window format for location {location_idx}. Skipping.")

    # Add time window constraints for vehicle start and end nodes (depot).
    depot_time_window = time_windows[depot]
    for vehicle_id in range(num_vehicle):
        # Apply depot time window to the start of each vehicle's route.
        time_dimension.SetCumulVarRange(
            routing.Start(vehicle_id), depot_time_window[0], depot_time_window[1])
        # Apply depot time window to the end of each vehicle's route.
        time_dimension.SetCumulVarRange(
            routing.End(vehicle_id), depot_time_window[0], depot_time_window[1])
        # Minimize the end time of each vehicle to encourage shorter routes.
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(vehicle_id)))

    # 5. Add Pickup and Delivery constraints.
    for pickup_node, delivery_node in pickups_deliveries:
        pickup_index = manager.NodeToIndex(pickup_node)
        delivery_index = manager.NodeToIndex(delivery_node)
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure pickup happens before delivery.
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= 
            time_dimension.CumulVar(delivery_index))

    # 6. Set search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(30) # Set a time limit for the solver

    # 7. Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # 8. Get the objective value.
    if solution:
        obj = solution.ObjectiveValue()
    else:
        obj = -1 # Indicate no solution found

    return obj
