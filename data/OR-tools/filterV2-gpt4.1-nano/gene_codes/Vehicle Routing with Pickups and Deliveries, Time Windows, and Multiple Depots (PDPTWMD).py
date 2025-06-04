# Vehicle Routing with Pickups and Deliveries, Time Windows, and Multiple Depots (PDPTWMD)
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, starts: list, ends: list):
    # Create the data model as a dictionary to hold all problem data
    data = {
        'time_matrix': time_matrix,  # Matrix representing travel times between locations
        'time_windows': time_windows,  # List of time windows for each location
        'pickups_deliveries': pickups_deliveries,  # List of pickup and delivery pairs
        'num_vehicles': num_vehicle,  # Number of vehicles available
        'starts': starts,  # Starting nodes for each vehicle
        'ends': ends,  # Ending nodes for each vehicle
    }

    # Create the routing index manager to handle node indices and vehicle routing
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Instantiate the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        # Convert routing indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return travel time between nodes
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set the cost of each arc to the travel time between nodes
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to model time with waiting times and maximum travel time
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (in time units)
        1440,  # Maximum total time per vehicle (e.g., 24 hours)
        False,  # Do not force start cumul to zero
        'Time')  # Dimension name
    time_dimension = routing.GetDimensionOrDie('Time')  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to index
        # Set the feasible time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add pickup and delivery constraints to ensure correct pairing and sequencing
    for pickup, delivery in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(pickup)  # Convert pickup node
        delivery_index = manager.NodeToIndex(delivery)  # Convert delivery node
        # Add pickup and delivery pair constraints
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
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

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1