from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import numpy as np
from typing import List, Dict, Any
from scipy.spatial import distance_matrix
import matplotlib.pyplot as plt

def plot_graph(coords, partial_solutions):
    colors = ['r','g', 'b', 'c', 'm', 'y','darkorange','navy']
    # plot scatter plot without dummy node
    n_node = len(coords)-1
    plt.figure(figsize=(10,10))
    plt.scatter(
        coords[1:, 0], coords[1:, 1], s=[50 for _ in range(n_node)]
    )
    for i, partial_solution in enumerate(partial_solutions):
        # mark the inital node
        plt.plot(
            coords[partial_solution[0], 0],
            coords[partial_solution[0], 1],
            "x",
            markersize=10,
        )

        # connect the visited nodes
        for idx in range(len(partial_solution) - 1):
            node, next_node = partial_solution[idx], partial_solution[idx + 1]
            plt.plot(
                [coords[node, 0], coords[next_node, 0]],
                [coords[node, 1], coords[next_node, 1]],
                colors[i],
                lw=2,
                alpha=0.8,
            )

        # connect the initial node and the last node if terminated
        last, first = partial_solution[-1], partial_solution[0]
        plt.plot(
            [coords[last, 0], coords[first, 0]],
            [coords[last, 1], coords[first, 1]],
            colors[i],
            lw=2,
            alpha=0.8,
        )
    plt.show()


def get_graph_mat(n_node=10, map_size=3, dim=2, n_vehicle=2):
    # Generate fully connected graph
    # 0~n_vehicle index node is dummy nodes for ortools.
    starting_points = [[0, 0], [3, 3], [0, 3], [3, 0], [0, 1], [3, 2], [0, 2], [3, 1]]
    coords = map_size * np.random.uniform(size=(n_node+n_vehicle, dim))
    # adding start points test num vehicle = 2
    for i in range(n_vehicle):
        coords = np.vstack([coords, np.array(starting_points[i])])
        coords[i] = np.array(starting_points[i])
    # num of coords  == n_nude + n_vehicle + 1(dummy)
    dist_mat = distance_matrix(coords, coords)
    return coords, dist_mat

def create_data_model(dist_matrix_list, n_vehicle):
    "Stores the data for the problem"
    data = {}
    num_node = len(dist_matrix_list[0])
    data['distance_matrix'] = dist_matrix_list
    data['num_vehicles'] = n_vehicle
    data['starts'] = sorted([num_node-i-1 for i in range(data['num_vehicles'])])
    print(data['starts'])
    data['ends'] = [i for i in range(data['num_vehicles'])]
    print(data['ends'])
    return data

def make_partial_soultion(data, manager, routing):
    partial_solutions = []
    for vehicle_id in range(data['num_vehicles']):
        partial_solution = []
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            if manager.IndexToNode(index) !=0:
                partial_solution.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        partial_solutions.append(partial_solution)
    return partial_solutions

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while (not routing.IsEnd(index)):
            plan_output += ' {}'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            plan_output += ' -> '
        plan_output += '\nDistance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))

if __name__ == "__main__" :
    n_node = 40
    n_vehicle = 8
    coords, dist_mat = get_graph_mat(n_node=n_node, n_vehicle=n_vehicle)
    dist_mat_list =(dist_mat * 100).astype(np.int64)
    data = create_data_model(dist_mat_list.tolist(), n_vehicle)

    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'],
                                           data['starts'],
                                           data['ends'],)

    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,
        1200,
        True,
        dimension_name
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)
    partial_soultions = make_partial_soultion(data, manager, routing)
    print(partial_soultions)
    if solution:
        print_solution(data,manager,routing,solution)
    plot_graph(coords, partial_soultions)
    print(1)