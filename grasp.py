from random import randrange
import numpy
from copy import copy, deepcopy
import sys
import time

class GreedyRandomizedAdaptiveSearchProcedure:
    def __init__(self, tsp_file, best_solution): 
        self.alpha = 0.5
        self.tsp_file = tsp_file
        self.best_solution = self.convert_from_true_false_matrix(best_solution)
        self.best_solution_cost = this.compute_cost(this.best_solution)

    def grasp(self):
        start_time = time.time()
        elapsed_time = 0

        while elapsed_time <= 60:
            candidate_solution = this.greedy_randomized_construction()
            candidate_solution = this.local_search(candidate_solution)
            candidate_solution_cost = this.compute_cost(candidate_solution)

            if candidate_solution_cost < self.best_solution_cost:
                self.best_solution = candidate_solution
                self.best_solution_cost = candidate_solution_cost
            elapsed_time = time.time() - start_time

        return self.convert_from_integer_array_to_true_false_matrix(elf.best_solution)

    def greedy_randomized_construction(self):
        candidate_solution


    def local_search(self, candidate_solution):

    def line_intersection(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

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
