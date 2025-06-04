# Capacitated Vehicle Routing Problem with Time Windows and Open Start and End locations (OCVRPTW)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(time_matrix: list, time_windows: list, demands: list, vehicle_capacities: list, num_vehicle: int):
    # Initialize data model
    data = {}
    data["time_matrix"] = time_matrix  # Matrix representing travel times between locations
    data["time_windows"] = time_windows  # Time windows for each location
    data["demands"] = demands  # Demand at each location
    data["vehicle_capacities"] = vehicle_capacities  # Capacity of each vehicle
    data["num_vehicles"] = num_vehicle  # Total number of vehicles
    data["depot"] = 0  # Depot index, not used in open routes but required by the model

    # Create the routing index manager with correct arguments
    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["depot"])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Transit callback for travel times
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)  # Convert routing index to node index
        to_node = manager.IndexToNode(to_index)  # Convert routing index to node index
        return data["time_matrix"][from_node][to_node]  # Return travel time between nodes

    transit_callback_index = routing.RegisterTransitCallback(time_callback)  # Register callback

    # Set cost of each arc to travel time
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows dimension to model
    time = "Time"
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        30,  # Allow waiting time at locations
        30,  # Maximum time per vehicle route
        False,  # Don't force start cumul to zero
        time,  # Dimension name
    )
    time_dimension = routing.GetDimensionOrDie(time)  # Retrieve the time dimension

    # Set time window constraints for each location
    for location_idx, time_window in enumerate(data["time_windows"]):
        index = manager.NodeToIndex(location_idx)  # Convert node to routing index
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])  # Set time window

    # Set time window constraints for each vehicle start node
    for v in range(data["num_vehicles"]):
        start_index = routing.Start(v)  # Get start index for vehicle
        time_dimension.CumulVar(start_index).SetRange(
            data["time_windows"][0][0], data["time_windows"][0][1]
        )  # Set start time window

    # Add demand dimension for capacity constraints
    def demand_callback(from_index):
        node = manager.IndexToNode(from_index)  # Convert routing index to node index
        return data["demands"][node]  # Return demand at node

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)  # Register demand callback
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  # Demand callback index
        0,  # Null capacity slack
        data["vehicle_capacities"],  # Vehicle capacities
        True,  # Start cumul to zero
        "Capacity",  # Dimension name
    )

    # Removed the invalid SetStartEnd loop to fix the AttributeError

    # Set search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC  # Strategy for initial solution
    )

    # Solve the routing problem with the specified parameters
    solution = routing.SolveWithParameters(search_parameters)

    # Return the objective value if a solution is found, otherwise return -1
    if solution:
        return solution.ObjectiveValue()
    else:
        return -1