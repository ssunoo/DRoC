# Capacitated Vehicle Routing Problem with Time Windows and Resource Constraints (CVRPTWRC)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicles: int, depot: int, vehicle_load_time: int, vehicle_unload_time: int, depot_capacity: int):
    # Create the data model as a dictionary to store all problem data
    data = {}
    data["time_matrix"] = time_matrix  # Travel time between locations
    data["time_windows"] = time_windows  # Allowed time windows for each location
    data["demands"] = demands  # Demand at each location
    data["vehicle_capacities"] = vehicle_capacities  # Capacity of each vehicle
    data["num_vehicles"] = num_vehicles  # Total number of vehicles
    data["depot"] = depot  # Index of the depot location
    data["vehicle_load_time"] = vehicle_load_time  # Loading time per vehicle
    data["vehicle_unload_time"] = vehicle_unload_time  # Unloading time per vehicle
    data["depot_capacity"] = depot_capacity  # Capacity constraint at the depot

    # Create the routing index manager to handle node indices
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["depot"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert from routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert to node index
        return data["time_matrix"][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set arc cost evaluator for all vehicles to minimize total travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Dimension to model to handle time windows and waiting times
    time = "Time"
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at nodes (max wait time)
        30,  # Maximum time per vehicle route
        False,  # Do not force start cumul to zero
        time,  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location (except depot)
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == data["depot"]:
            continue  # Skip depot as it is handled separately
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node (depot)
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        # Set the start time window for each vehicle at the depot
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][data["depot"]][0],  # Earliest start time at depot
            data["time_windows"][data["depot"]][1],  # Latest start time at depot
        )

    # Add demand dimension to handle capacity constraints
    def demand_callback(from_index):
        node = manager.IndexToNode(from_index)  # Convert routing index to node index
        return data["demands"][node]  # Return demand at the node

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)  # Register demand callback
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  # Demand callback index
        0,  # Null slack capacity
        data["vehicle_capacities"],  # Vehicle capacities
        True,  # Start cumul to zero
        "Capacity",  # Dimension name
    )

    # Add resource constraints at the depot for load and unload operations
    solver = routing.solver()  # Get the solver instance
    intervals = []  # List to hold intervals for resource constraints
    for i in range(data["num_vehicles"]):
        # Add load interval for each vehicle
        load_interval = solver.FixedDurationIntervalVar(
            time_dimension.CumulVar(routing.Start(i)),  # Start time of load
            data["vehicle_load_time"],  # Duration of load operation
            f"load_{i}",  # Name of the load interval
        )
        # Add unload interval for each vehicle
        unload_interval = solver.FixedDurationIntervalVar(
            time_dimension.CumulVar(routing.End(i)),  # End time of unload
            data["vehicle_unload_time"],  # Duration of unload operation
            f"unload_{i}",  # Name of the unload interval
        )
        intervals.extend([load_interval, unload_interval])  # Add to intervals list
    # Add cumulative resource constraint at the depot to limit concurrent load/unload operations
    depot_usage = [1 for _ in intervals]  # Usage of resource per interval
    solver.Add(solver.Cumulative(intervals, depot_usage, data["depot_capacity"], "DepotResource"))  # Add constraint

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy to find initial solution

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1