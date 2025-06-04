
# Vehicle Routing Problem with Time Windows and Resource Constraints (VRPTWRC)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve():
    # Data initialization
    time_matrix = [
        [0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 6, 7],
        [6, 0, 8, 2, 3, 7, 9, 7, 9, 2, 5, 9, 10, 10, 5],
        [9, 8, 0, 11, 2, 2, 2, 1, 9, 10, 11, 2, 2, 6, 9],
        [8, 2, 11, 0, 5, 10, 10, 9, 5, 1, 10, 10, 7, 8, 6],
        [7, 3, 2, 5, 0, 8, 9, 7, 9, 8, 10, 8, 8, 5, 10],
        [3, 7, 2, 10, 8, 0, 6, 6, 8, 9, 8, 2, 2, 2, 1],
        [6, 9, 2, 10, 9, 6, 0, 1, 3, 7, 7, 6, 6, 5, 6],
        [2, 7, 1, 9, 7, 6, 1, 0, 2, 3, 2, 6, 8, 4, 6],
        [3, 9, 9, 5, 9, 8, 3, 2, 0, 2, 6, 7, 8, 5, 3],
        [2, 2, 10, 1, 8, 9, 7, 3, 2, 0, 4, 6, 5, 6, 3],
        [6, 5, 11, 10, 10, 8, 7, 2, 6, 4, 0, 5, 6, 3, 7],
        [6, 9, 2, 10, 8, 2, 6, 6, 7, 6, 5, 0, 2, 2, 1],
        [4, 10, 2, 7, 8, 2, 6, 8, 8, 5, 6, 2, 0, 2, 6],
        [6, 10, 6, 8, 5, 2, 5, 4, 5, 6, 3, 2, 2, 0, 2],
        [7, 5, 9, 6, 10, 1, 6, 6, 3, 3, 7, 1, 6, 2, 0],
    ]
    num_vehicles = 4
    depot = 0
    time_windows = [
        (0, 0),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
        (10, 11),
    ]
    vehicle_load_time = 0
    vehicle_unload_time = 0
    depot_capacity = 0

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(time_matrix), num_vehicles, depot
    )

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        return time_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,  # transit callback index
        30,  # allow waiting time
        30,  # vehicle maximum travel time
        False,  # start cumul to zero
        time,  # dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location
    for location_idx, time_window in enumerate(time_windows):
        if location_idx == depot:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for the depot.
    index = manager.NodeToIndex(depot)
    time_dimension.CumulVar(index).SetRange(time_windows[0][0], time_windows[0][1])

    # Add resource constraints at the depot
    # Create and register a resource callback.
    def depot_resource_callback(from_index, to_index):
        return vehicle_load_time if manager.IndexToNode(from_index) == depot else vehicle_unload_time

    depot_resource_callback_index = routing.RegisterUnaryTransitCallback(depot_resource_callback)
    routing.AddConstantDimensionWithCapacity(
        depot_resource_callback_index,  # callback index
        0,  # slack var
        depot_capacity,  # capacity
        True,  # fix start cumul to zero
        "DepotCapacity",  # dimension name
    )

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print(f"Objective: {solution.ObjectiveValue()}")
        time_dimension = routing.GetDimensionOrDie(time)
        total_time = 0
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            plan_output = f"Route for vehicle {vehicle_id}:\n" 
            route_time = 0
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                plan_output += f"{manager.IndexToNode(index)} Time({solution.Min(time_var)}, {solution.Max(time_var)}) -> " 
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_time += time_matrix[manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
            time_var = time_dimension.CumulVar(index)
            plan_output += f"{manager.IndexToNode(index)} Time({solution.Min(time_var)}, {solution.Max(time_var)})\n" 
            plan_output += f"Time of the route: {route_time}min\n" 
            print(plan_output)
            total_time += route_time
        print(f"Total time of all routes: {total_time}min")
        return 0
    else:
        return -1
