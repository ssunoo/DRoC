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
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback with service times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        travel_time = time_matrix[from_node][to_node]
        # Add service time at the from_node if it's not the depot
        service = 0
        if from_node != depot:
            service = service_time[from_node]
        return travel_time + service

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting
        10000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Set time window constraints for all locations
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set depot time window for all vehicles
    for vehicle_id in range(num_vehicle):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(time_windows[depot][0], time_windows[depot][1])

    # Add Pickup and Delivery constraints
    for pickup, delivery in pickups_deliveries:
        pickup_idx = manager.NodeToIndex(pickup)
        delivery_idx = manager.NodeToIndex(delivery)
        routing.AddPickupAndDelivery(pickup_idx, delivery_idx)
        routing.solver().Add(routing.VehicleVar(pickup_idx) == routing.VehicleVar(delivery_idx))
        routing.solver().Add(time_dimension.CumulVar(pickup_idx) <= time_dimension.CumulVar(delivery_idx))

    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.FromSeconds(30)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Return objective value
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1