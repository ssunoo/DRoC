# Vehicle Routing Problem with Time Windows and Multiple Depots (VRPTWMD)
# This code solves a VRPTWMD using Google OR-Tools
# It considers multiple depots, time windows, and vehicle constraints
# Import necessary modules from OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicles: int):
    # Create data model containing problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'starts': starts,  # Starting nodes for each vehicle
        'ends': ends,  # Ending nodes for each vehicle
        'num_vehicles': num_vehicles,  # Number of vehicles
        'depot': 0  # Index of the depot, assuming a single depot for simplicity
    }

    # Create the routing index manager to manage node indices for vehicles
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set arc cost evaluator for all vehicles to minimize total travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension to model to handle time windows and waiting times
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (max wait time)
        30,  # Maximum time per vehicle route
        False,  # Do not force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location (except depot)
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for the location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node to match depot's time window
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)  # Get start node index for vehicle
        depot_time_window = data['time_windows'][data['depot']]  # Depot's time window
        # Set the start node's time window to match depot's time window
        time_dimension.CumulVar(start_index).SetRange(depot_time_window[0], depot_time_window[1])

    # Minimize the total time by setting finalizer for each vehicle's start and end nodes
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))  # Minimize start node time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))  # Minimize end node time

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC as the first solution strategy
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the VRPTWMD problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution is found, compute total travel time
    if solution:
        total_time = 0  # Initialize total time
        for vehicle_id in range(data['num_vehicles']):  # For each vehicle
            index = routing.Start(vehicle_id)  # Start at the vehicle's start node
            while not routing.IsEnd(index):  # Traverse the route until end node
                previous_index = index  # Store current index
                index = solution.Value(routing.NextVar(index))  # Move to next node
                node_index = manager.IndexToNode(index)  # Convert to node index
                prev_node_index = manager.IndexToNode(previous_index)  # Previous node index
                # Accumulate travel time between previous and current node
                total_time += data['time_matrix'][prev_node_index][node_index]
        return total_time  # Return total travel time of the solution
    else:
        return -1  # Return -1 if no solution is found
