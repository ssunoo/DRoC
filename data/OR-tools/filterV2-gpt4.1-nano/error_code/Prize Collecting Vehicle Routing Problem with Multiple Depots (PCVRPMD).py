# Prize Collecting Vehicle Routing Problem with Multiple Depots (PCVRPMD)
# This code aims to solve a vehicle routing problem where multiple depots are involved, and prizes are collected at nodes.
# It uses Google OR-Tools to model and solve the problem.

# Import necessary modules from OR-Tools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve(distance_matrix: list, prizes: list, max_distance: int, num_vehicle: int, starts: list, ends: list):
    # Ensure starts and ends are lists of integers
    if not isinstance(starts, list):
        starts = [starts]
    if not isinstance(ends, list):
        ends = [ends]

    # Convert starts and ends to list of integers if they are not
    starts = [int(s) for s in starts]
    ends = [int(e) for e in ends]

    # Create the routing index manager with correct arguments
    # The manager maps the problem nodes to internal indices used by the solver
    # It takes the number of nodes, number of vehicles, start nodes, and end nodes
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicle, starts, ends)

    # Create the routing model which will hold the routing problem
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback to compute distances between nodes
    def distance_callback(from_index, to_index):
        # Convert from routing variable indices to node indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Return the distance between the nodes from the distance matrix
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Set the arc cost evaluator for all vehicles to the distance callback
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add a dimension to track the total distance traveled by each vehicle
    routing.AddDimension(
        transit_callback_index,  # Transit callback index
        0,  # No slack (waiting time)
        max_distance,  # Maximum distance a vehicle can travel
        True,  # Start cumul to zero
        'Distance')  # Dimension name

    # Retrieve the distance dimension for further constraints or analysis
    distance_dimension = routing.GetDimensionOrDie('Distance')

    # Add disjunctions with penalties for nodes that are not start or end nodes
    # This encourages visiting nodes by assigning prizes as penalties
    for node in range(len(distance_matrix)):
        if node not in starts and node not in ends:
            # Add a disjunction with a penalty equal to the prize value
            # This means the solver can choose to skip visiting this node at a cost
            routing.AddDisjunction([manager.NodeToIndex(node)], prizes[node])

    # Set search parameters for the solver
    # First solution heuristic and local search metaheuristic are specified
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(15)  # Limit search time to 15 seconds

    # Solve the routing problem with the specified parameters
    assignment = routing.SolveWithParameters(search_parameters)

    # Initialize total prize collected
    total_prize = 0
    if assignment:
        # Loop through all nodes in the routing solution
        for node in range(routing.Size()):
            index = node
            # Check if the node is not a start or end node
            if not routing.IsStart(index) and not routing.IsEnd(index):
                # Check if the node is visited in the solution
                if assignment.Value(routing.NextVar(index)) != index:
                    # Add the prize value for visited nodes
                    total_prize += prizes[manager.IndexToNode(index)]
    # Return the total prize collected in the solution
    return total_prize