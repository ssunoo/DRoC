# Vehicle Routing with Pickups and Deliveries and Time Windows (PDPTW)
# This code models and solves a Vehicle Routing Problem with pickups, deliveries, and time windows using OR-Tools.
# It aims to find optimal routes for a fleet of vehicles to service a set of locations within specified time constraints.

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'pickups_deliveries': pickups_deliveries,  # Pairs of pickup and delivery locations
        'num_vehicles': num_vehicle,  # Number of vehicles available
        'depot': depot  # Starting location for all vehicles
    }

    # Create the routing index manager to manage node indices
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times between locations
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles to minimize total travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension to model time constraints
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at locations (slack)
        30,  # Maximum time per vehicle (capacity)
        False,  # Do not force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot as it is handled separately
        # Check if time window is valid
        if time_window and len(time_window) == 2:
            start, end = time_window
            # Validate that start time is less than or equal to end time
            if start <= end:
                index = manager.NodeToIndex(location_idx)  # Convert node to index
                # Set the allowable time range for the location
                time_dimension.CumulVar(index).SetRange(start, end)

    # Add time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        depot_time_window = data['time_windows'][data['depot']]  # Depot's time window
        if depot_time_window and len(depot_time_window) == 2:
            start, end = depot_time_window
            if start <= end:
                # Set the start node's time window
                time_dimension.CumulVar(index).SetRange(start, end)

    # Add Pickup and Delivery constraints to ensure correct pairing and order
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])  # Pickup location index
        delivery_index = manager.NodeToIndex(request[1])  # Delivery location index
        routing.AddPickupAndDelivery(pickup_index, delivery_index)  # Add pickup-delivery pair
        # Ensure pickup and delivery are served by the same vehicle
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        # Enforce that pickup occurs before delivery
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Minimize the total travel time by setting finalizers for each vehicle's start and end nodes
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))  # Start node
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))  # End node

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC strategy for initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Total travel time of the solution
    else:
        return -1  # Indicate failure to find a solution
