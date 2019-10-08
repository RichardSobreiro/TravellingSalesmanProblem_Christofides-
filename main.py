import os
from os import listdir
from os.path import isfile, join
import sys
import numpy as np
import time
import matplotlib.pyplot as plt
from TSPFile import TSPFile
from PrimAlgorithm import PrimAlgorithm
from copy import copy, deepcopy

# Flag that controls if intermediary plots showing each steps of the 
# Christofides should be printed or not
# Flag que controle se gráficos intermediários mostrando cada etapa do 
# algortimo de Christofides devem ser exibidos ou não
show_intermediary_steps_graphs = True
source_indexes_visited = []
best_destiny_indexes_visited = []

def main():
    # Read all instances and store it in an array of TSPFile objects
    tsp_files = compute_tsp_instances()

    # For each instance apply some heuristic based on christofides algorithm
    for tsp_file in tsp_files:
        pairs_visited = [False] * tsp_file.dimension
        for i in range(tsp_file.dimension):
            pairs_visited[i] = [False] * tsp_file.dimension

        start_time = time.time()

        euler_cycle = christofides(tsp_file)

        variable_neighborhood_descent(tsp_file, euler_cycle, 50, 0, pairs_visited)

        elapsed_time = time.time() - start_time

        show_final_solution(tsp_file, euler_cycle, elapsed_time)

def compute_tsp_instances():
    tsp_files = np.array([])

    mypath = 'Instances/EUC_2D'
    tsp_files = np.append(tsp_files, [read_files_in_directory(mypath)])

    mypath = 'Instances'
    tsp_files = np.append(tsp_files, [read_files_in_directory(mypath)])

    return tsp_files

def read_files_in_directory(mypath):
    tsp_files = np.array([])
    filenames = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    for filename in filenames:
        tsp_file = TSPFile()
        name, ext = os.path.splitext(filename)
        if ext == '.tsp':
            filepath = mypath + '/' + filename
            f = open(filepath)
            line = f.readline()
            while line:
                param = ''
                value = ''
                if ':' in line:
                    [param, value] = line.split(':')
                else :
                    param = line
                tsp_file.fill_props(param, value)
                line = f.readline()
            f.close()
            # After closing the file the adjacency matrix should be computed
            # Após fechar o arquivo a matriz de adjacência deve ser computada
            tsp_file.compute_adjacency_matrix()
            tsp_files = np.append(tsp_files, [tsp_file])
    return tsp_files

def christofides(tsp_file):
    start_time = time.time()
# ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # 1 - Calculate minimum spanning tree T
    # 1 - Calculo da árvore geradora mínima através do algoritmo de PRIM
    minimum_spanning_tree = compute_minimum_spanning_tree(tsp_file)
    
    if show_intermediary_steps_graphs == True:
        plt.scatter(tsp_file.x_coord, tsp_file.y_coord)
        for i in range(tsp_file.dimension):
            plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
            if minimum_spanning_tree[i] != -1:
                plt.plot([tsp_file.x_coord[i], tsp_file.x_coord[minimum_spanning_tree[i]]],[tsp_file.y_coord[i], tsp_file.y_coord[minimum_spanning_tree[i]]],'k-')
        plt.title(tsp_file.name + ' - MST')
        plt.figure()
# ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # 2 -  Calculate the set of vertices O with odd degree in T
    odd_degree_nodes = find_odd_degree_nodes(tsp_file, minimum_spanning_tree)

    if show_intermediary_steps_graphs == True:
        for i in range(tsp_file.dimension):
            if i in odd_degree_nodes:
                plt.plot(tsp_file.x_coord[i], tsp_file.y_coord[i], 'ro')
                plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
        plt.title(tsp_file.name + ' - Odd Degree')
        plt.figure()
# ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # 3 - Form the subgraph of G using only the vertices of O
    subgraph_odd_degree_nodes = subgraph_only_odd_degree_nodes(tsp_file, odd_degree_nodes)
# ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # 4 - Construct a minimum-weight perfect matching M in this subgraph
    matching  = construct_minimum_weight_perfect_matching(tsp_file, subgraph_odd_degree_nodes)
    
    if show_intermediary_steps_graphs == True:
        for i in range(tsp_file.dimension):
            if i in odd_degree_nodes:
                plt.plot(tsp_file.x_coord[i], tsp_file.y_coord[i], 'ro')
                plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
                for j in range(tsp_file.dimension):
                    if matching[i][j] == True:
                        plt.plot([tsp_file.x_coord[i], tsp_file.x_coord[j]],[tsp_file.y_coord[i], tsp_file.y_coord[j]],'k-')
        plt.title(tsp_file.name + ' - Perfect Matching')
        plt.figure()

# ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # 5 - Unite matching and spanning tree T ∪ M to form an Eulerian multigraph
    spanning_tree_union_matching = minimum_matching_union_spanning_tree(tsp_file, matching, minimum_spanning_tree)

    if show_intermediary_steps_graphs == True:
        for i in range(tsp_file.dimension):
            plt.plot(tsp_file.x_coord[i], tsp_file.y_coord[i], 'ro')
            plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
            for j in range(tsp_file.dimension):
                if spanning_tree_union_matching[i][j] == True:
                    plt.plot([tsp_file.x_coord[i], tsp_file.x_coord[j]],[tsp_file.y_coord[i], tsp_file.y_coord[j]],'k-')
        plt.title(tsp_file.name + ' - MST U Perfect Matching')
        plt.figure()
# ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # 6 - Calculate Euler tour and Remove repeated vertices
    euler_cycle  = compute_euler_tour(tsp_file, spanning_tree_union_matching)

    elapsed_time = time.time() - start_time

    return euler_cycle

def variable_neighborhood_descent(tsp_file, euler_cycle, neighborhoods_max, time_limit, pairs_visited):
    n = 0
    current_cost = 0
    best_cost = euler_tour_cost(tsp_file, euler_cycle)

    while n <= neighborhoods_max:
        i = i + 1
        euler_cycle_neighbor = choose_most_improving_neighbour(tsp_file, euler_cycle, pairs_visited)
        current_cost = euler_tour_cost(tsp_file, euler_cycle_neighbor)
        if current_cost < best_cost:
            best_cost = current_cost
            euler_cycle = euler_cycle_neighbor

def euler_tour_cost(tsp_file, euler_cycle):
    cost = 0
    for i in range(tsp_file.dimension):
        for j in range(tsp_file.dimension):
            if euler_cycle[i][j] == True:
                cost += tsp_file.adjacency_matrix[i][j]
    return cost

def choose_most_improving_neighbour(tsp_file, euler_cycle, pairs_visited):
    index_1 = 0
    index_2 = 0
    best_index_destiny_1 = 0
    best_index_source_2 = 0
    max_distance = 0
    for i in range(tsp_file.dimension):
        for j in range(tsp_file.dimension):
            if (euler_cycle[i][j] == True) and (max_distance < tsp_file.adjacency_matrix[i][j]):
                max_distance = tsp_file.adjacency_matrix[i][j]
                index_1 = i
                index_2 = j

    min_distance_index_1 = sys.maxsize
    for j in range(tsp_file.dimension):
        if (euler_cycle[i][j] == False) and (tsp_file.adjacency_matrix[index_1][j] < min_distance_index_1) and (pairs_visited[index_1][j] == False):
            min_distance_index_1 = tsp_file.adjacency_matrix[index_1][j]
            best_index_destiny_1 = j

    euler_cycle_neighbor = deepcopy(euler_cycle)
    euler_cycle_neighbor[index_1][index_2] = False
    euler_cycle_neighbor[index_1][best_index_destiny_1] = True

    best_index_destiny_1_destiny = 0
    for i in range(tsp_file.dimension):
        for j in range(tsp_file.dimension):
            if (best_index_destiny_1 == i) and (euler_cycle[i][j] == True):
                euler_cycle_neighbor[i][j] = False
                best_index_destiny_1_destiny = j
            if (best_index_destiny_1 == j) and (euler_cycle[i][j] == True):
                euler_cycle_neighbor[i][j] = False
                best_index_destiny_1_destiny = j
            
    euler_cycle_neighbor[best_index_destiny_1][index_2] = True
    euler_cycle_neighbor[index_2][best_index_destiny_1_destiny] = True

    pairs_visited[index_1][best_index_destiny_1] = True
    return euler_cycle_neighbor



def show_final_solution(tsp_file, euler_cycle, elapsed_time):
    cost = 0
    for i in range(tsp_file.dimension):
        plt.plot(tsp_file.x_coord[i], tsp_file.y_coord[i], 'ro')
        plt.grid(True)
        plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
        for j in range(tsp_file.dimension):
            if euler_cycle[i][j] == True:
                cost += tsp_file.adjacency_matrix[i][j]
                plt.plot([tsp_file.x_coord[i], tsp_file.x_coord[j]],[tsp_file.y_coord[i], tsp_file.y_coord[j]],'k-')
    plt.title(tsp_file.name + ' - Algorithm\'s output (Cost = '+ str(cost) + '| Elapsed time = ' + str(round(elapsed_time, 5)) + ')')

    plt.show()

def compute_minimum_spanning_tree(tsp_file):
    prim_algorithm = PrimAlgorithm(tsp_file.adjacency_matrix)
    minimum_spanning_tree = prim_algorithm.primMST()
    del prim_algorithm
    return minimum_spanning_tree

def find_odd_degree_nodes(tsp_file, minimum_spanning_tree):
    odd_degree_nodes = []
    for i in range(tsp_file.dimension):
        count_degree = 0
        for j in range(tsp_file.dimension):
            if (minimum_spanning_tree[j] == i) or (minimum_spanning_tree[i] == j):
                count_degree += 1
        if (count_degree % 2) == 1:
            odd_degree_nodes.append(i)
    return odd_degree_nodes

def subgraph_only_odd_degree_nodes(tsp_file, odd_degree_nodes):
    subgraph_odd_degree_nodes = [0] * tsp_file.dimension
    for i in range(tsp_file.dimension):
        subgraph_odd_degree_nodes[i] = [0] * tsp_file.dimension

    for i in range(tsp_file.dimension):
        for j in range(tsp_file.dimension):
            if (i not in odd_degree_nodes) or (j not in odd_degree_nodes):
                subgraph_odd_degree_nodes[i][j] = 0
                subgraph_odd_degree_nodes[j][i] = subgraph_odd_degree_nodes[i][j]
            else:
                subgraph_odd_degree_nodes[i][j] = tsp_file.adjacency_matrix[i][j]
    return subgraph_odd_degree_nodes

def construct_minimum_weight_perfect_matching(tsp_file, subgraph_odd_degree_nodes):
    matching  = [False] * tsp_file.dimension
    for i in range(tsp_file.dimension):
        matching[i] = [False] * tsp_file.dimension

    for i in range(tsp_file.dimension):
        min = sys.maxsize
        match_node = -1
        for j in range(tsp_file.dimension):
            if subgraph_odd_degree_nodes[i][j] > 0 and subgraph_odd_degree_nodes[i][j] < min and matching[i][j] == False:
                min = subgraph_odd_degree_nodes[i][j]
                match_node = j
        if match_node > 0:
            matching[i][match_node] = True
    return matching

def minimum_matching_union_spanning_tree(tsp_file, matching, minimum_spanning_tree):
    spanning_tree_union_matching = [False] * tsp_file.dimension
    for i in range(tsp_file.dimension):
        spanning_tree_union_matching[i] = [False] * tsp_file.dimension

    for i in range(tsp_file.dimension):
        for j in range(tsp_file.dimension):
            if matching[i][j] == True:
                spanning_tree_union_matching[i][j] = True
            if (minimum_spanning_tree[i] == j):
                spanning_tree_union_matching[i][j] = True
    return spanning_tree_union_matching

def compute_euler_tour(tsp_file, spanning_tree_union_matching):
    euler_cycle  = [False] * tsp_file.dimension
    for i in range(tsp_file.dimension):
        euler_cycle[i] = [False] * tsp_file.dimension

    nodes_already_visited = np.array([]).astype(int)
    current_node = 0
    i = 0
    while i < tsp_file.dimension:
        nodes_already_visited = np.append(nodes_already_visited, [current_node])
        min_distance = sys.maxsize
        node_min_distance = -1
        for j in range(tsp_file.dimension):
            if (current_node != j) \
            and (spanning_tree_union_matching[current_node][j] == True \
                or spanning_tree_union_matching[j][current_node] == True) \
            and tsp_file.adjacency_matrix[current_node][j] < min_distance \
            and (j not in nodes_already_visited):
                min_distance = tsp_file.adjacency_matrix[current_node][j]
                node_min_distance = j
        if node_min_distance != -1:
            euler_cycle[current_node][node_min_distance] = True
            current_node = node_min_distance
        else:
            min_distance = sys.maxsize
            node_min_distance = -1
            for j in range(tsp_file.dimension):
                if (current_node != j) \
                and tsp_file.adjacency_matrix[current_node][j] < min_distance \
                and (j not in nodes_already_visited):
                    min_distance = tsp_file.adjacency_matrix[current_node][j]
                    node_min_distance = j
            if node_min_distance != -1:
                euler_cycle[current_node][node_min_distance] = True
                current_node = node_min_distance
            else:
                euler_cycle[current_node][0] = True
                current_node = 0
        i += 1
    return euler_cycle

if __name__ == '__main__':
    main()