# Capacitated Vehicle Routing Problem with Time Windows and Open Start and End locations (OCVRPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int):
    # Initialize data model
    data = {}
    data["time_matrix"] = time_matrix  # Matrix representing travel times between locations
    data["time_windows"] = time_windows  # Allowed time windows for each location
    data["demands"] = demands  # Demand at each location
    data["vehicle_capacities"] = vehicle_capacities  # Capacity of each vehicle
    data["num_vehicles"] = num_vehicle  # Total number of vehicles
    data["starts"] = list(range(len(time_matrix)))  # Start nodes for each vehicle
    data["ends"] = list(range(len(time_matrix)))  # End nodes for each vehicle

    # Create the routing index manager to manage the conversion between node indices and routing indices
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["starts"], data["ends"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times between locations
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return data["time_matrix"][from_node][to_node]  # Return travel time between nodes

    # Register the transit callback with the routing model
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Set arc cost evaluator for all vehicles to minimize travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension to handle time constraints
    time = "Time"
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at locations (slack)
        30,  # Maximum time per vehicle (capacity)
        False,  # Don't force start cumul to zero
        time,  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        # Set the allowed time window for each location
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Set time window constraints for each vehicle start node
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)  # Get start node index for vehicle
        # Set the allowed time window for the start node of each vehicle
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][data["starts"][vehicle_id]][0],
            data["time_windows"][data["starts"][vehicle_id]][1],
        )

    # Add demand dimension to handle capacity constraints
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        return data["demands"][from_node]  # Return demand at the node

    # Register demand callback
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    # Add capacity dimension with vehicle capacities
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  # Demand callback index
        0,  # Null capacity slack
        data["vehicle_capacities"],  # Vehicle maximum capacities
        True,  # Start cumul to zero
        "Capacity",  # Dimension name
    )

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC strategy for initial solution
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem with the specified search parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1
