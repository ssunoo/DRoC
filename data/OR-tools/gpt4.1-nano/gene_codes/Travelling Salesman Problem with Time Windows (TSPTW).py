# Travelling Salesman Problem with Time Windows (TSPTW)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, depot: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,  # Matrix representing travel times between locations
        'time_windows': time_windows,  # List of time windows for each location
        'num_vehicles': 1,  # Number of vehicles (here, only one vehicle)
        'depot': depot,  # Index of the depot location
    }

    # Create the routing index manager to handle node indices
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert to node index
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set arc cost evaluator for all vehicles to minimize travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time dimension to model to handle time windows and waiting times
    time = 'Time'  # Name of the dimension
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (slack)
        30,  # Maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero
        time,  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location (except depot)
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for the location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for the start node of each vehicle
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        # Set the time window for the vehicle start node (depot)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0], data['time_windows'][data['depot']][1]
        )

    # Minimize the final arrival times for each vehicle
    for i in range(data['num_vehicles']):
        # Add finalizer to minimize the start time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        # Add finalizer to minimize the end time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use the PATH_CHEAPEST_ARC strategy for initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Initialize objective value
    obj = -1
    if solution:
        # If a solution is found, get the objective value (total travel time)
        obj = solution.ObjectiveValue()
    return obj  # Return the objective value (total travel time or -1 if no solution)