# Vehicle Routing Problem with Time Windows and Resource Constraints (VRPTWRC)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, num_vehicle: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int, depot: int):
    # Create the data model as a dictionary to hold all problem data
    data = {}
    data["time_matrix"] = time_matrix  # Travel time between locations
    data["time_windows"] = time_windows  # Allowed time windows for each location
    data["num_vehicles"] = num_vehicle  # Number of vehicles
    data["depot"] = depot  # Index of the depot location
    data["vehicle_load_time"] = vehicle_load_time  # Loading time at depot
    data["vehicle_unload_time"] = vehicle_unload_time  # Unloading time at depot
    data["depot_capacity"] = depot_capacity  # Capacity constraint at the depot

    # Create the routing index manager to manage conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["depot"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return data["time_matrix"][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set cost of each arc to be the travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint dimension
    time = "Time"  # Name of the dimension
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        60,  # Allow waiting time at nodes (slack)
        60,  # Maximum time per vehicle (capacity of time dimension)
        False,  # Do not force start cumul to zero
        time,  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the dimension

    # Add time window constraints for each location except the depot
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for the location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)  # Get start index for vehicle
        # Set the depot's time window for the start node
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][data["depot"]][0], data["time_windows"][data["depot"]][1]
        )

    # Add resource constraints at the depot using the solver
    solver = routing.solver()  # Get the solver from the routing model
    intervals = []  # List to hold interval variables for resource constraints
    for i in range(data["num_vehicles"]):
        # Add fixed duration interval variable for vehicle load time at start
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),  # Start time of route
                data["vehicle_load_time"],  # Duration of load time
                "depot_interval",  # Name of the interval
            )
        )
        # Add fixed duration interval variable for vehicle unload time at end
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),  # End time of route
                data["vehicle_unload_time"],  # Duration of unload time
                "depot_interval",  # Name of the interval
            )
        )
    # Create a list of usage counts for each interval (all are 1 since each interval uses one resource)
    depot_usage = [1 for _ in range(len(intervals))]
    # Add cumulative resource constraint to limit total usage at the depot
    solver.Add(
        solver.Cumulative(intervals, depot_usage, data["depot_capacity"], "depot")
    )

    # Minimize the total travel time by finalizing the start and end times for each vehicle
    for i in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))  # Minimize start time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))  # Minimize end time

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy to find initial solution

    # Solve the VRPTWRC problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Total travel time of the solution
    else:
        return -1  # Indicate no solution found