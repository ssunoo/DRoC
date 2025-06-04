# Vehicle Routing with Pickups and Deliveries, Time Windows, Multiple Depots, and Duration Limit (PDPTWMDL)
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list, duration_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing variables
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, starts, ends)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to time matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    # Register the transit callback and get its index
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc (edge) for the routing problem
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint to ensure service occurs within specified time windows
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time at nodes
        duration_limit,  # maximum route duration for each vehicle
        False,  # Don't force start cumul to zero; can be set to True if needed
        time)  # Dimension name
    # Retrieve the time dimension to add further constraints
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, (start, end) in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        # Set the allowed time window for each node
        time_dimension.CumulVar(index).SetRange(start, end)

    # Add pickup and delivery constraints to enforce pickup-delivery pairs
    for pickup, delivery in pickups_deliveries:
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(manager.NodeToIndex(pickup), manager.NodeToIndex(delivery))
        # Ensure that pickup and delivery are assigned to the same vehicle
        routing.solver().Add(routing.VehicleVar(manager.NodeToIndex(pickup)) == routing.VehicleVar(manager.NodeToIndex(delivery)))
        # Enforce that pickup occurs before delivery in time
        routing.solver().Add(time_dimension.CumulVar(manager.NodeToIndex(pickup)) <= time_dimension.CumulVar(manager.NodeToIndex(delivery)))

    # Set the search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC heuristic for initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1