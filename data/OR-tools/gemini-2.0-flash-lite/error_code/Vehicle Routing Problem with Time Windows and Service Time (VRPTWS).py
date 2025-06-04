
# Vehicle Routing Problem with Time Windows and Service Time (VRPTWS)
# [('user', 'The generated code cannot run or time out.')]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Define the solve function
def solve():
    from ortools.constraint_solver import routing_enums_pb2
    from ortools.constraint_solver import pywrapcp

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["locations"]),
        data["num_vehicles"],
        data["depot"],
    )

    # Create the routing model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index, 0, 3000, True, dimension_name
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define the penalty for each node.
    penalty = 1000
    for node in range(1, len(data["locations"])):  # Exclude depot
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Setting the time windows.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(time_callback_index, 30, 30, False, "Time")
    time_dimension = routing.GetDimensionOrDie("Time")
    for i in range(len(data["time_windows"])):
        time_dimension.CumulVar(manager.NodeToIndex(i)).SetRange(data["time_windows"][i][0], data["time_windows"][i][1])

    # Setting the soft time windows.
    for i in range(len(data["time_windows"])):
        # Set a soft lower bound for the time windows with a cost of 100.
        time_dimension.SetCumulVarSoftLowerBound(manager.NodeToIndex(i), 100, 100) # Added coefficient argument

    # Setting the search parameters.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.seconds = 10

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    def print_solution(solution, manager):
        """Prints solution on console."""
        print(f"Objective: {solution.ObjectiveValue()}")
        total_distance = 0
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            plan_output = f"Route for vehicle {vehicle_id}:\n"  # Corrected line
            route_distance = 0
            while not routing.IsEnd(index):
                plan_output += f" {manager.IndexToNode(index)} ->"
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            plan_output += f" {manager.IndexToNode(index)}\n"
            plan_output += f"Distance of the route: {route_distance}m\n"
            print(plan_output)
            total_distance += route_distance
        print(f"Total distance of all routes: {total_distance}m")

    if solution:
        print_solution(solution, manager)

    return solution



data = {
    "locations": [
        (45, 30),
        (45, 60),
        (45, 90),
        (45, 120),
        (45, 150),
        (45, 180),
        (45, 210),
        (45, 240),
        (45, 270),
        (45, 300),
        (45, 330),
        (45, 360),
        (45, 390),
        (45, 420),
        (45, 450),
        (45, 480),
        (45, 510),
        (45, 540),
        (45, 570),
        (45, 600),
    ],
    "distance_matrix": [
        [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420, 450, 480, 510, 540, 570],
        [30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420, 450, 480, 510, 540],
        [60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420, 450, 480, 510],
        [90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420, 450, 480],
        [120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420, 450],
        [150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420],
        [180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390],
        [210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360],
        [240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330],
        [270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300],
        [300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270],
        [330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210, 240],
        [360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180, 210],
        [390, 360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150, 180],
        [420, 390, 360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120, 150],
        [450, 420, 390, 360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90, 120],
        [480, 450, 420, 390, 360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60, 90],
        [510, 480, 450, 420, 390, 360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30, 60],
        [540, 510, 480, 450, 420, 390, 360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0, 30],
        [570, 540, 510, 480, 450, 420, 390, 360, 330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0],
    ],
    "num_vehicles": 4,
    "depot": 0,
    "time_windows": [
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
        (0, 30),
    ],
}

def time_callback(from_index, to_index):
    """Returns the travel time between the two nodes."""
    return data["distance_matrix"][from_index][to_index]



