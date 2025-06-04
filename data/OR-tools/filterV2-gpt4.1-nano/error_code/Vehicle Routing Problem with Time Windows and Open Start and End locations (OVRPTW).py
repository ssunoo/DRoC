# Vehicle Routing Problem with Time Windows and Open Start and End locations (OVRPTW)
# [('user', 'The generated code cannot run or time out.')] 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int):
    """
    Args:
        time_matrix: contains the integer travel times between locations
        time_windows: the list of tuples for time windows of the customers
        num_vehicle: the number of the vehicle

    Returns:
        obj: a number representing the objective value of the solution
    """
    # Create data model
    data = {}
    data["time_matrix"] = time_matrix
    data["time_windows"] = time_windows
    data["num_vehicles"] = num_vehicle
    # Set start and end locations for each vehicle (open routes)
    data["starts"] = [0] * num_vehicle
    data["ends"] = [0] * num_vehicle

    # Create Routing Index Manager
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["starts"], data["ends"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == 0:  # Assuming depot is at index 0
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set start and end time windows for each vehicle
    for vehicle_id in range(data["num_vehicles"]):
        start_index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        # Set start time window
        time_dimension.CumulVar(start_index).SetRange(
            data["time_windows"][data["starts"][vehicle_id]][0],
            data["time_windows"][data["starts"][vehicle_id]][1],
        )
        # Set end time window
        time_dimension.CumulVar(end_index).SetRange(
            data["time_windows"][data["ends"][vehicle_id]][0],
            data["time_windows"][data["ends"][vehicle_id]][1],
        )

    # Minimize the start and end times
    for i in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Set search parameters with a timeout
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.FromSeconds(30)  # set a 30 second timeout

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Check for solution feasibility
    if solution:
        return solution.ObjectiveValue()
    else:
        # Optionally, print debug info
        print("No solution found or infeasible.")
        return -1