# Vehicle Routing Problem with Time Windows and Multiple Depots (VRPTWMD)
# This code solves a VRPTWMD using Google OR-Tools.
# Import necessary modules from OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, starts: list, ends: list, num_vehicles: int):
    # Create the data model as a dictionary to hold problem data
    data = {}
    data["time_matrix"] = time_matrix  # Matrix representing travel times between locations
    data["time_windows"] = time_windows  # List of time windows for each location
    data["starts"] = starts  # List of start nodes for each vehicle
    data["ends"] = ends  # List of end nodes for each vehicle
    data["num_vehicles"] = num_vehicles  # Total number of vehicles

    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["starts"], data["ends"])

    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute travel times between nodes
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return data["time_matrix"][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set the arc cost evaluator for all vehicles to the transit callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a time dimension to model time constraints and waiting times
    time = "Time"
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at locations (slack)
        30,  # Maximum time per vehicle (capacity)
        False,  # Do not force start cumul to zero
        time,  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Add time window constraints for each location
    for location_idx, time_window in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node
    for vehicle_id in range(data["num_vehicles"]):
        start_index = routing.Start(vehicle_id)  # Get start index for vehicle
        start_node_idx = data["starts"][vehicle_id]  # Get start node index
        start_time_window = data["time_windows"][start_node_idx]  # Get start node's time window
        # Set the start node's time window
        time_dimension.CumulVar(start_index).SetRange(start_time_window[0], start_time_window[1])

    # Minimize the total travel time for each vehicle by adding finalizer
    for vehicle_id in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(vehicle_id)))  # Minimize start node time
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(vehicle_id)))  # Minimize end node time

    # Set search parameters with a time limit to find a good solution within constraints
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution
    search_parameters.time_limit.FromSeconds(30)  # Limit search to 30 seconds

    # Solve the VRPTWMD problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()  # Total travel time of the solution
    else:
        return -1  # No solution found within time limit