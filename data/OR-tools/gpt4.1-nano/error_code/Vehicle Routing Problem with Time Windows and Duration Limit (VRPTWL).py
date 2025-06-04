# Vehicle Routing Problem with Time Windows and Duration Limit (VRPTWL)
# This code solves a VRPTWL using Google OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, duration_limit: int):
    # Create the data model containing all problem data
    data = {
        'time_matrix': time_matrix,  # Travel time between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'num_vehicles': num_vehicle,  # Number of vehicles
        'depot': depot  # Starting and ending point for all vehicles
    }

    # Create the routing index manager to manage the conversion between node indices and internal routing indices
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles to minimize total travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension to model time constraints
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at locations
        duration_limit,  # Maximum route duration (makespan)
        False,  # Don't force start cumul to zero
        time  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the dimension

    # Add time window constraints for each location (except depot)
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for the location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        # Set the start time window to match depot's time window
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )

    # Add route duration constraints to limit the total time per route
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        # Minimize the start node's cumulative time variable
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index))
        end_index = routing.End(vehicle_id)
        # Minimize the end node's cumulative time variable
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(end_index))
        # Enforce maximum route duration (makespan) constraint
        routing.solver().Add(
            time_dimension.CumulVar(end_index) <= duration_limit
        )

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Optional: set a time limit for the search process (e.g., 30 seconds)
    search_parameters.time_limit.seconds = 30
    # Choose the first solution strategy
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the VRPTWL problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value (total travel time) if a solution is found
    if solution:
        return solution.ObjectiveValue()
    return -1  # Return -1 if no solution is found
