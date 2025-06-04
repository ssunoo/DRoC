# Vehicle Routing with Pickups and Deliveries, Time Windows, and Duration Limit (PDPTWL)
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int, duration_limit: int):
    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), num_vehicle, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Returns the travel time between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc (edge) in the route based on travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint to ensure service occurs within specified time windows
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time at nodes
        duration_limit,  # maximum route duration
        False,  # Don't force start cumul to zero
        'Time')  # Dimension name
    time_dimension = routing.GetDimensionOrDie('Time')

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        index = manager.NodeToIndex(location_idx)
        # Set the allowed time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add pickup and delivery constraints to enforce pickup-delivery pairs
    for pickup, delivery in pickups_deliveries:
        pickup_index = manager.NodeToIndex(pickup)
        delivery_index = manager.NodeToIndex(delivery)
        # Add pickup and delivery pair to the routing model
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        # Ensure that pickup and delivery are assigned to the same vehicle
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce that pickup occurs before delivery in the route
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index))

    # Set search parameters for the solver, including the heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1