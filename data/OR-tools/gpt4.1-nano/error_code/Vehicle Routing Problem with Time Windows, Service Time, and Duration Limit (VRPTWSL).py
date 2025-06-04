# Vehicle Routing Problem with Time Windows, Service Time, and Duration Limit (VRPTWSL)
# This code solves a VRPTW with additional constraints using Google OR-Tools.
# Import necessary modules from OR-Tools.
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, depot: int, service_time: list, duration_limit: int):
    # Instantiate the data model with all problem parameters.
    data = {
        'time_matrix': time_matrix,  # Travel time between locations.
        'time_windows': time_windows,  # Allowed time windows for each location.
        'num_vehicles': num_vehicle,  # Number of vehicles.
        'depot': depot,  # Starting and ending location for all vehicles.
        'service_time': service_time,  # Service time at each location.
        'duration_limit': duration_limit  # Maximum route duration.
    }

    # Create the routing index manager to manage the conversion between node indices and routing indices.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times plus service times.
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index.
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node] + data['service_time'][from_node]  # Travel time + service time.

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback.

    # Set the cost of each arc to the transit callback.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension to model to handle time constraints.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,  # Transit callback index.
        30,  # Allow waiting time at locations.
        data['duration_limit'],  # Maximum route duration.
        False,  # Do not force start cumul to zero.
        time  # Dimension name.
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the dimension.

    # Set time window constraints for each location.
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index.
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])  # Set time window.

    # Set time window constraints for each vehicle start and end node.
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)  # Start node index.
        end_index = routing.End(vehicle_id)  # End node index.
        # Set start node time window based on depot's time window.
        time_dimension.CumulVar(start_index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )
        # Set end node time window based on depot's time window.
        time_dimension.CumulVar(end_index).SetRange(
            data['time_windows'][data['depot']][0],
            data['time_windows'][data['depot']][1]
        )
        # Add route duration minimization constraints.
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(start_index))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(end_index))

    # Add a route duration upper bound for each vehicle.
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(vehicle_id)))

    # Set search parameters for the solver.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution.
    search_parameters.time_limit.seconds = 30  # Optional: set a time limit for the solver.

    # Solve the VRPTW problem with the specified parameters.
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1.
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1