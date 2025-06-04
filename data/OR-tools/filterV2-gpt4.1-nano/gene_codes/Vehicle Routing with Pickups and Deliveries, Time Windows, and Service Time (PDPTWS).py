# Vehicle Routing with Pickups and Deliveries, Time Windows, and Service Time (PDPTWS)
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
    # Create the data model
    data = {
        'time_matrix': time_matrix,
        'time_windows': time_windows,
        'pickups_deliveries': pickups_deliveries,
        'num_vehicles': num_vehicle,
        'depot': depot,
        'service_time': service_time
    }

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        180,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1])

    # Add Pickup and Delivery constraints
    for pickup_node, delivery_node in data['pickups_deliveries']:
        pickup_idx = manager.NodeToIndex(pickup_node)
        delivery_idx = manager.NodeToIndex(delivery_node)
        routing.AddPickupAndDelivery(pickup_idx, delivery_idx)
        routing.solver().Add(routing.VehicleVar(pickup_idx) == routing.VehicleVar(delivery_idx))
        routing.solver().Add(
            time_dimension.CumulVar(pickup_idx) <= time_dimension.CumulVar(delivery_idx))

    # Add Service Time at each node
    for node_idx, service in enumerate(data['service_time']):
        index = manager.NodeToIndex(node_idx)
        # Adjust the transit callback to include service time
        # Create a new transit callback that adds service time
        def service_time_callback(from_index, to_index, node_idx=node_idx):
            return time_callback(from_index, to_index) + data['service_time'][node_idx]
        # Register the new callback
        service_callback_index = routing.RegisterTransitCallback(service_time_callback)
        # Set the arc cost evaluator to include service time
        routing.SetArcCostEvaluatorOfAllVehicles(service_callback_index)

    # Set first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1