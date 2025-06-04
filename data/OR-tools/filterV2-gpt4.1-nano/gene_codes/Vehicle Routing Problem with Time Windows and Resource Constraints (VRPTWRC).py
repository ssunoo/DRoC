# Vehicle Routing Problem with Time Windows and Resource Constraints (VRPTWRC)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int, depot: int):
    # Create the data model
    data = {
        'time_matrix': time_matrix,  # Matrix representing travel times between locations
        'time_windows': time_windows,  # Allowed time windows for each location
        'num_vehicles': num_vehicle,  # Number of vehicles available
        'vehicle_load_time': vehicle_load_time,  # Loading time per vehicle
        'vehicle_unload_time': vehicle_unload_time,  # Unloading time per vehicle
        'depot_capacity': depot_capacity,  # Capacity constraint at the depot
        'depot': depot,  # Index of the depot node
    }

    # Create the routing index manager to manage node indices for vehicles
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert to node index
        return data['time_matrix'][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension to model
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        60,  # Allow waiting time at nodes (slack)
        60,  # Maximum total time per vehicle
        False,  # Do not force start cumul to zero
        time,  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the dimension

    # Add time window constraints for each location except depot
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for the location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        # Set the start time window for the vehicle
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][data['depot']][0],  # Start of depot time window
            data['time_windows'][data['depot']][1],  # End of depot time window
        )

    # Add resource constraints at the depot using solver
    solver = routing.solver()
    intervals = []  # List to hold interval variables for resource constraints
    for i in range(data['num_vehicles']):
        # Add fixed duration interval variable at the start of each route
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),  # Start time of route
                data['vehicle_load_time'],  # Duration of loading
                'depot_interval',  # Name of interval
            )
        )
        # Add fixed duration interval variable at the end of each route
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),  # End time of route
                data['vehicle_unload_time'],  # Duration of unloading
                'depot_interval',  # Name of interval
            )
        )
    depot_usage = [1 for _ in range(len(intervals))]  # Usage indicator for each interval
    # Add cumulative resource constraint at the depot
    solver.Add(
        solver.Cumulative(intervals, depot_usage, data['depot_capacity'], 'depot')
    )

    # Minimize the total travel time by finalizing start and end variables
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))  # Minimize start time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))  # Minimize end time

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy to find initial solution
    )

    # Solve the VRPTWRC problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Total travel time of the solution
    else:
        return -1  # Indicate no solution found
