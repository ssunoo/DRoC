# Vehicle Routing with Pickups and Deliveries, Time Windows, Multiple Depots, and Duration Limit (PDPTWMDL)
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list, duration_limit: int):
    # Create the data model dictionary containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'num_vehicles': num_vehicle,  # Number of vehicles
        'starts': starts,  # Starting nodes for each vehicle
        'ends': ends,  # Ending nodes for each vehicle
        'time_windows': time_windows,  # Allowed time windows at each location
        'pickups_deliveries': pickups_deliveries,  # List of pickup-delivery pairs
        'duration_limit': duration_limit  # Maximum route duration per vehicle
    }

    # Create the routing index manager to handle node indices and vehicle assignments
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Convert from routing variable Index to actual node index in the time matrix
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc to be the travel time between nodes
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to model time with waiting times and route duration limits
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        data['duration_limit'],  # Maximum route duration
        False,  # Do not force start cumul to zero
        'Time')  # Dimension name
    time_dimension = routing.GetDimensionOrDie('Time')  # Retrieve the time dimension

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to index in routing model
        # Set the time window for the location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add pickup and delivery constraints to ensure correct pairing and sequencing
    for pickup, delivery in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(pickup)  # Convert pickup node
        delivery_index = manager.NodeToIndex(delivery)  # Convert delivery node
        routing.AddPickupAndDelivery(pickup_index, delivery_index)  # Add pickup-delivery pair
        # Ensure both pickup and delivery are served by the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce that pickup occurs before delivery in time
        routing.solver().Add(time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index))

    # Set the search parameters for the solver, choosing a heuristic for initial solution
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)  # Strategy to find initial solution

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize objective value
    obj = -1
    # If a solution is found, extract the objective value
    if solution:
        obj = solution.ObjectiveValue()

    # Return the objective value of the solution
    return obj
