
# Vehicle Routing with Pickups and Deliveries, Time Windows, and Duration Limit (PDPTWL)
# [('user', 'The solution failed the code execution test: Error: name \'solve\' is not defined\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmp7qujckqg.py", line 105, in <module>\n    result = solve(time_matrix, time_windows, pickups_deliveries, num_vehicle, depot, duration_limit)\nNameError: name \'solve\' is not defined\n\n')]
import sys
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve_pdptwl(time_matrix, num_vehicle, depot, duration_limit, time_windows, pickups_deliveries):
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

    # Add Time Windows and Duration Limit constraint.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        sys.maxsize,  # allow waiting time (slack_max) - set to a very large number
        duration_limit,  # maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location.
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot).
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            time_windows[depot][0], time_windows[depot][1]
        )

    # Add Pickups and Deliveries constraint.
    for request in pickups_deliveries:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Same vehicle constraint
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        # Pickup before delivery constraint (using time dimension)
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
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
    # Set a time limit for the search to prevent indefinite runs.
    # The value (e.g., 30 seconds) can be adjusted based on problem size and desired solution quality.
    search_parameters.time_limit.seconds = 30

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get the objective value.
    obj = None
    if solution:
        obj = solution.ObjectiveValue()

    return obj
