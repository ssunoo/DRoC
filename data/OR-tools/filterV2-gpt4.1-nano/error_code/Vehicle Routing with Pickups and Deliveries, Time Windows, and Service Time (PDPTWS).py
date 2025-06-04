# Vehicle Routing with Pickups and Deliveries, Time Windows, and Service Time (PDPTWS)
# [('user', 'The solution failed the code execution test: Error: RoutingModel.AddConstantDimension() takes 5 positional arguments but 6 were given\nTraceback: Traceback (most recent call last):\n  File "/tmp/tmppu3n3pix.py", line 86, in <module>\n    result = solve(time_matrix, time_windows, pickups_deliveries, num_vehicle, depot, service_time)\n  File "/tmp/tmppu3n3pix.py", line 49, in solve\n    routing.AddConstantDimension(\nTypeError: RoutingModel.AddConstantDimension() takes 5 positional arguments but 6 were given\n\n')] 
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int, service_time: list):
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to time matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        'Time')
    time_dimension = routing.GetDimensionOrDie('Time')

    # Set time window for each location
    for location_idx, (start, end) in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(start, end)

    # Add pickup and delivery constraints
    for pickup, delivery in pickups_deliveries:
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        routing.solver().Add(time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index))

    # Add service time at each node
    for node_idx, service in enumerate(service_time):
        index = manager.NodeToIndex(node_idx)
        time_dimension.CumulVar(index).SetRange(time_windows[node_idx][0], time_windows[node_idx][1])
        # Add service time as a fixed delay
        routing.AddConstantDimension(
            service,
            0,
            0,
            True,
            'Service')
        service_dimension = routing.GetDimensionOrDie('Service')
        service_dimension.CumulVar(index).SetValue(service)

    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return routing.Cost(solution)
    else:
        return -1