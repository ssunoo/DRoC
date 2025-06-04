# Vehicle Routing with Pickups and Deliveries and Time Windows (PDPTW)
# Import necessary modules from OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, pickups_deliveries: list, num_vehicle: int, depot: int):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time matrix between locations
        'time_windows': time_windows,  # Time windows for each location
        'pickups_deliveries': pickups_deliveries,  # List of pickup and delivery pairs
        'num_vehicles': num_vehicle,  # Number of vehicles
        'depot': depot  # Index of the depot node
    }

    # Create the routing index manager to manage node indices
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a time dimension to model time constraints
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        30,  # Maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location except the depot
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Validate the time window
        if time_window[0] <= time_window[1]:
            try:
                # Set the allowable time window for the node
                time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
            except Exception as e:
                # Handle exceptions if setting the range fails
                print(f"Failed to set time window for node {location_idx}: {e}")
                return -1  # Return error code
        else:
            # Invalid time window, start time is after end time
            print(f"Invalid time window for node {location_idx}: {time_window}")
            return -1  # Return error code

    # Add time window constraints for each vehicle start node to match depot's time window
    depot_time_window = data['time_windows'][data['depot']]
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        try:
            # Set the start node's time window to match depot's time window
            time_dimension.CumulVar(index).SetRange(depot_time_window[0], depot_time_window[1])
        except Exception as e:
            # Handle exceptions if setting the range fails
            print(f"Failed to set depot time window for vehicle {vehicle_id}: {e}")
            return -1  # Return error code

    # Add pickup and delivery constraints
    for pickup_node, delivery_node in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(pickup_node)  # Convert pickup node
        delivery_index = manager.NodeToIndex(delivery_node)  # Convert delivery node
        routing.AddPickupAndDelivery(pickup_index, delivery_index)  # Add pickup-delivery pair
        # Ensure pickup and delivery are served by the same vehicle
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        # Enforce pickup occurs before delivery
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )

    # Minimize the total travel time for all vehicles
    for vehicle_id in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(vehicle_id)))  # Minimize start time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(vehicle_id)))  # Minimize end time

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy to find initial solution

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found
    if solution:
        return solution.ObjectiveValue()  # Total minimized travel time
    else:
        return -1  # No solution found, return error code