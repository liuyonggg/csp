import unittest
import math

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

class CreateDistanceCallback(object):
  """Create callback to calculate distances between points."""

  def __init__(self, locations):
    """Initialize distance array."""
    size = len(locations)
    self.matrix = {}

    for from_node in xrange(size):
      self.matrix[from_node] = {}
      for to_node in xrange(size):
        if from_node == to_node:
          self.matrix[from_node][to_node] = 0
        else:
          x1 = locations[from_node][0]
          y1 = locations[from_node][1]
          x2 = locations[to_node][0]
          y2 = locations[to_node][1]
          self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

  def Distance(self, from_node, to_node):
    return self.matrix[from_node][to_node]

class CreateDemandCallback(object):
  """Create callback to get demands at each location."""
  def __init__(self, demands):
    self.matrix = demands
  def Demand(self, from_node, to_node):
    return self.matrix[from_node]

class TestVRP(unittest.TestCase):
    def setUp(self):
        pass

    def solve(self, routing, locations, demands, dimension_name, dist_callback, vehicle_capacity=100, null_capacity_slack=0, depot = 0):
        assert (routing)
        routing.SetDepot(depot);
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

        demands_at_locations = CreateDemandCallback(demands)
        demands_callback = demands_at_locations.Demand
        # Adding capacity dimension constraints.
        fix_start_cumul_to_zero = True
        routing.AddDimension(demands_callback, null_capacity_slack, vehicle_capacity,
                             fix_start_cumul_to_zero, dimension_name)
        
        
        # Solve, displays a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)
        return assignment

        
    def test_one_dimention(self):
        locations = [[0,0], [2,0], [4,0], [8,0], [16,0], [32,0], [64,0], [128,0], [256,0]]
        demands = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        depot = 0
        num_vehicles = 1

        routing = pywrapcp.RoutingModel(len(locations), num_vehicles)
        dist_between_locations = CreateDistanceCallback(locations)
        dist_callback = dist_between_locations.Distance
        assignment = self.solve(routing, locations, demands, "Capacity", dist_callback)

        assert (assignment)
        prev_node_index = -1
        route_demand = -1
        for vehicle_nbr in range(num_vehicles):
            index = routing.Start(vehicle_nbr)
            prev_node_index = routing.IndexToNode(index)
            route = ''
            route_dist = 0
            route_demand = 0

            while not routing.IsEnd(index):
                node_index = routing.IndexToNode(index)
                route += str(node_index) + " -> "
                route_dist += dist_callback(prev_node_index, node_index)
                route_demand += demands[node_index]
                prev_node_index = routing.IndexToNode(index)
                index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        route += str(node_index)
        route_dist += dist_callback(prev_node_index, node_index)
        route_demand += demands[node_index]

        self.assertEqual(route, '0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 0')
        self.assertEqual(route_dist, 512)
        self.assertEqual(route_demand, 36)

    def test_one_dimention_two_cars(self):
        locations = [[0,0], [2,0], [4,0], [8,0], [16,0], [32,0], [64,0], [128,0], [256,0]]
        demands = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        depot = 0
        num_vehicles = 2

        routing = pywrapcp.RoutingModel(len(locations), num_vehicles)
        dist_between_locations = CreateDistanceCallback(locations)
        dist_callback = dist_between_locations.Distance
        assignment = self.solve(routing, locations, demands, "Capacity", dist_callback)

        assert (assignment)
        prev_node_index = -1
        for vehicle_nbr in range(num_vehicles):
            index = routing.Start(vehicle_nbr)
            prev_node_index = routing.IndexToNode(index)
            route = ''
            route_dist = 0
            route_demand = 0

            while not routing.IsEnd(index):
                node_index = routing.IndexToNode(index)
                route += str(node_index) + " -> "
                route_dist += dist_callback(prev_node_index, node_index)
                route_demand += demands[node_index]
                prev_node_index = routing.IndexToNode(index)
                index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        route += str(node_index)
        route_dist += dist_callback(prev_node_index, node_index)
        route_demand += demands[node_index]

        self.assertEqual(route, '0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 0')
        self.assertEqual(route_dist, 512)
        self.assertEqual(route_demand, 36)


    def test_one_dimention_branch_two_cars(self):
        locations = [[0,0], [2,0], [4,0], [8,0], [16,0], [32,0], [64,0], [128,0], [256,0],
                            [-2,0], [-4,0], [-8,0], [-16,0], [-32,0], [-64,0], [-128,0], [-256,0]
                    ]
        demands = range(len(locations))
        depot = 0
        num_vehicles = 2

        routing = pywrapcp.RoutingModel(len(locations), num_vehicles)
        dist_between_locations = CreateDistanceCallback(locations)
        dist_callback = dist_between_locations.Distance
        assignment = self.solve(routing, locations, demands, "Capacity", dist_callback)
        exp_route = ["0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 0", "0 -> 9 -> 10 -> 11 -> 12 -> 13 -> 14 -> 15 -> 16 -> 0"]
        exp_dist = [512, 512]
        exp_demand = [36, 100]

        assert (assignment)
        prev_node_index = -1
        for vehicle_nbr in range(num_vehicles):
            index = routing.Start(vehicle_nbr)
            prev_node_index = routing.IndexToNode(index)
            route = ''
            route_dist = 0
            route_demand = 0

            while not routing.IsEnd(index):
                node_index = routing.IndexToNode(index)
                route += str(node_index) + " -> "
                route_dist += dist_callback(prev_node_index, node_index)
                route_demand += demands[node_index]
                prev_node_index = routing.IndexToNode(index)
                index = assignment.Value(routing.NextVar(index))

            node_index = routing.IndexToNode(index)
            route += str(node_index)
            route_dist += dist_callback(prev_node_index, node_index)
            route_demand += demands[node_index]
            #print "vehicle: %d, route: %s, route_dist: %d, route_demand: %d" % (vehicle_nbr, route, route_dist, route_demand)
            self.assertEqual(route, exp_route[vehicle_nbr])
            self.assertEqual(route_dist, exp_dist[vehicle_nbr])
            self.assertEqual(route_demand, exp_demand[vehicle_nbr])

    def test_two_dimention_1(self):
        pass
    def test_two_dimention_2(self):
        pass

if __name__ == '__main__':
    unittest.main()

