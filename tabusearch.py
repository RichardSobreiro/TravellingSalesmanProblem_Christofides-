from random import randrange
import numpy
from copy import copy, deepcopy
import sys

class TabuSearch:
    def __init__(self, memory_size, tsp_file, current_solution): 
        self.memory_size = memory_size
        self.tsp_file = tsp_file
        self.current_solution = self.convert_from_true_false_matrix(current_solution)
        self.visited_solutions = [0] * self.memory_size
        for i in range(memory_size):
            self.visited_solutions[i] = deepcopy(self.current_solution)

    def tabu_search(self):
        i = 0
        current_best_neighbor = deepcopy(self.current_solution)
        while i < self.memory_size:
            neighbor_solution = self.generate_neighbor_solution(current_best_neighbor)
            if self.solution_not_already_visited(neighbor_solution) == True:
                self.visited_solutions[i] = neighbor_solution
                current_best_neighbor = neighbor_solution
            i += 1
        
        best_neighbor = self.get_best_neighbor_solution()

        self.has_sub_tour(best_neighbor)

        return self.convert_from_integer_array_to_true_false_matrix(best_neighbor)
    
    def generate_neighbor_solution(self, current_best_neighbor):
        generated_solutions = [0] * 5
        generated_solutions_cost = [0] * 5
        for i in range(5):
            generated_solutions[i] = deepcopy(current_best_neighbor)

        smaller_cost = sys.maxsize
        index_smaller_cost = 0
        for i in range(5):
            nodes = numpy.random.random_integers(0, (self.tsp_file.dimension - 1), size=(1, 1))
            
            node_1 = nodes[0][0]
            sucessor_node_1 = generated_solutions[i][node_1]
            sucessor_sucessor_node_1 = generated_solutions[i][sucessor_node_1]
            antecessor_node_1 = -1
            for j in range(self.tsp_file.dimension):
                if generated_solutions[i][j] == node_1:
                    antecessor_node_1 = j
                    break
            
            generated_solutions[i][node_1] = sucessor_sucessor_node_1
            generated_solutions[i][sucessor_node_1] = node_1
            generated_solutions[i][antecessor_node_1] = sucessor_node_1

            next = 0
            for j in range(self.tsp_file.dimension):
                generated_solutions_cost[i] += self.tsp_file.adjacency_matrix[next][generated_solutions[i][next]]
                next = generated_solutions[i][next]
            if generated_solutions_cost[i] < smaller_cost:
                smaller_cost = generated_solutions_cost[i]
                index_smaller_cost = i
            
        return generated_solutions[index_smaller_cost]

    def has_sub_tour(self, solution):
        next = 0
        j = 1
        while j <= self.tsp_file.dimension:
            next = solution[next]
            if(next == 0):
                break
            j += 1

        if j != self.tsp_file.dimension:
            return True
        else:
            return False

    def get_best_neighbor_solution(self):
        smaller_cost = sys.maxsize
        index_smaller_cost = 0
        for i in range(self.memory_size):
            current_solution_cost = self.compute_cost(self.visited_solutions[i])
            if(current_solution_cost < smaller_cost):
                smaller_cost = current_solution_cost
                index_smaller_cost = i
        return self.visited_solutions[index_smaller_cost]

    def compute_cost(self, solution):
        cost = 0
        next = 0
        for i in range(self.tsp_file.dimension):
            cost += self.tsp_file.adjacency_matrix[next][solution[next]]
            next = solution[next]
        return cost

    def solution_not_already_visited(self, solution):
        not_visited = True
        for i in range(self.memory_size):
            if(numpy.array_equal(self.visited_solutions[i], solution)):
                not_visited = False
                break
        return not_visited

    def convert_from_true_false_matrix(self, true_false_matrix):
        next = 0
        result = [0] * self.tsp_file.dimension
        for i in range(self.tsp_file.dimension):
            for j in range(self.tsp_file.dimension):
                if true_false_matrix[next][j] == True:
                    result[next] = j
                    next = j
                    break
        return result

    def convert_from_integer_array_to_true_false_matrix(self, integer_array):
        result = [False] * self.tsp_file.dimension
        for i in range(self.tsp_file.dimension):
            result[i] = [False] * self.tsp_file.dimension
        
        for i in range(self.tsp_file.dimension):
            for j in range(self.tsp_file.dimension):
                if integer_array[i] == j:
                    result[i][j] = True
                    break
        return result
