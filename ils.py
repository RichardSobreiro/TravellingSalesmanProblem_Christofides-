import random
import numpy
from copy import copy, deepcopy
import sys
import time

class ils:
    def __init__(self, tsp_file, best_solution): 
        self.tsp_file = tsp_file
        self.best_solution = self.convert_from_true_false_matrix(best_solution)
        self.best_solution_cost = self.compute_cost(self.best_solution)
        self.begin_end_nodes_history = { }

    def ils(self):
        iterations = 0

        while iterations <= 500:
            candidate_solution = self.perturbation(self.best_solution)
            candidate_solution = self.local_search(candidate_solution)
            candidate_solution_cost = self.compute_cost(candidate_solution)

            if candidate_solution_cost < self.best_solution_cost:
                self.best_solution = candidate_solution
                self.best_solution_cost = candidate_solution_cost

            iterations += 1

        return self.convert_from_integer_array_to_true_false_matrix(self.best_solution)

    def perturbation(self, best_solution):
        new_candidate_solution = deepcopy(best_solution)
        begin_node_cut = -1
        end_node_cut = -1
        nodes_in_window = []

        begin_end_nodes_pair_not_used_yet = False
        while begin_end_nodes_pair_not_used_yet == False:
            nodes = numpy.random.random_integers(4, 20, size=(1, 1))
            window_cut_size = nodes[0][0]
            
            nodes = numpy.random.random_integers(0, (self.tsp_file.dimension - 1), size=(1, 1))
            begin_node_cut = nodes[0][0]
            current_node = new_candidate_solution[begin_node_cut]

            nodes_in_window.append(current_node)
            for i in range(window_cut_size):
                next_node = new_candidate_solution[current_node]
                new_candidate_solution[current_node] = -1
                nodes_in_window.append(next_node)
                current_node = next_node
            
            end_node_cut = nodes_in_window[window_cut_size]
            nodes_in_window.pop(window_cut_size)

            if (begin_node_cut in self.begin_end_nodes_history) and self.begin_end_nodes_history[begin_node_cut] == end_node_cut:
                begin_end_nodes_pair_not_used_yet = False
            else:
                begin_end_nodes_pair_not_used_yet = True                

        self.begin_end_nodes_history[begin_node_cut] = end_node_cut

        current_node = new_candidate_solution[begin_node_cut]
        nodes_already_linked = []
        while len(nodes_in_window) != 0:
            index_closest_node_in_window = -1
            min_distance = sys.maxsize
            for i in range(len(nodes_in_window)):
                possible_next_node = nodes_in_window[i]
                distance = self.tsp_file.adjacency_matrix[current_node][possible_next_node]
                if distance > 0 and distance < min_distance and \
                    new_candidate_solution[nodes_in_window[i]] == -1:
                    index_closest_node_in_window = i
                    min_distance = distance
            
            if index_closest_node_in_window != -1:
                if nodes_in_window[index_closest_node_in_window] in nodes_already_linked:
                    current_node = nodes_in_window[index_closest_node_in_window]
                    nodes_in_window.pop(index_closest_node_in_window)
                else:
                    new_candidate_solution[current_node] = nodes_in_window[index_closest_node_in_window] 
                    nodes_already_linked.append(nodes_in_window[index_closest_node_in_window])
                    nodes_in_window.pop(0)
                    if len(nodes_in_window) > 0:
                        current_node = nodes_in_window[0]
            else:
                if len(nodes_in_window) == 1:
                    new_candidate_solution[current_node] = end_node_cut
                    current_node = end_node_cut
                    nodes_in_window.pop(0)
                else:
                    break

        if self.has_sub_tour(new_candidate_solution):
            return self.best_solution
        else:
            return new_candidate_solution

    def local_search(self, candidate_solution):
        generated_solutions = [0] * 20
        generated_solutions_cost = [0] * 20
        for i in range(20):
            generated_solutions[i] = deepcopy(candidate_solution)

        smaller_cost = sys.maxsize
        index_smaller_cost = 0
        for i in range(20):
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

    def compute_cost(self, solution):
        cost = 0
        next = 0
        for i in range(self.tsp_file.dimension):
            cost += self.tsp_file.adjacency_matrix[next][solution[next]]
            next = solution[next]
        return cost

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
