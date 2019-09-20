from os import listdir
from os.path import isfile, join
import sys
import matplotlib.pyplot as plt
from TSPFile import TSPFile
from PrimAlgorithm import PrimAlgorithm

tsp_files = []

def main():
    read_files_compute_minimum_path()

def find_odd_degree_nodes(tsp_file, minimum_spanning_tree):
    odd_degree_nodes = []
    for i in range(tsp_file.dimension):
        count_degree = 0
        for j in range(tsp_file.dimension):
            if minimum_spanning_tree[j] == i:
                count_degree += 1
        if (count_degree % 2) == 1:
            odd_degree_nodes.append(i)
    return odd_degree_nodes

def read_files_compute_minimum_path():
    mypath = 'Instances/EUC_2D'
    filenames = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    for filename in filenames:
        tsp_file = TSPFile()
        filepath = 'Instances/EUC_2D/' + filename
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

        tsp_file.compute_adjacency_matrix()

        # Calculate minimum spanning tree T
        prim_algorithm = PrimAlgorithm(tsp_file.adjacency_matrix)
        minimum_spanning_tree = prim_algorithm.primMST()

        del prim_algorithm

        plt.scatter(tsp_file.x_coord, tsp_file.y_coord)
        for i in range(tsp_file.dimension):
            plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
            if minimum_spanning_tree[i] != -1:
                plt.plot([tsp_file.x_coord[i], tsp_file.x_coord[minimum_spanning_tree[i]]],[tsp_file.y_coord[i], tsp_file.y_coord[minimum_spanning_tree[i]]],'k-')
        plt.title(tsp_file.name + ' - MST')
        plt.figure()

        # Calculate the set of vertices O with odd degree in T
        odd_degree_nodes = find_odd_degree_nodes(tsp_file, minimum_spanning_tree)

        for i in range(tsp_file.dimension):
            if i in odd_degree_nodes:
                plt.plot(tsp_file.x_coord[i], tsp_file.y_coord[i], 'ro')
                plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
        plt.title(tsp_file.name + ' - Odd Degree')
        plt.figure()

        # Form the subgraph of G using only the vertices of O
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
        
        #Construct a minimum-weight perfect matching M in this subgraph
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
        
        for i in range(tsp_file.dimension):
            if i in odd_degree_nodes:
                plt.plot(tsp_file.x_coord[i], tsp_file.y_coord[i], 'ro')
                plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
                for j in range(tsp_file.dimension):
                    if matching[i][j] == True:
                        plt.plot([tsp_file.x_coord[i], tsp_file.x_coord[j]],[tsp_file.y_coord[i], tsp_file.y_coord[j]],'k-')
        plt.title(tsp_file.name + ' - Perfect Matching')
        plt.figure()

        # Unite matching and spanning tree T ∪ M to form an Eulerian multigraph

        plt.show()
        #tsp_files.append(tsp_file)

if __name__ == '__main__':
    main()