from os import listdir
from os.path import isfile, join
import matplotlib.pyplot as plt
from TSPFile import TSPFile
from Prim import PrimAlgorithm

tsp_files = []

def main():
    read_files_compute_adjacency_matrix()

def read_files_compute_adjacency_matrix():
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

        prim_algorithm = PrimAlgorithm(tsp_file.adjacency_matrix)
        minimum_spanning_tree = prim_algorithm.primMST()

        del prim_algorithm

        plt.scatter(tsp_file.x_coord, tsp_file.y_coord)
        for i in range(tsp_file.dimension):
            plt.annotate(str(i), (tsp_file.x_coord[i], tsp_file.y_coord[i]))
        plt.title(tsp_file.name)
        plt.show()
        tsp_files.append(tsp_file)

if __name__ == '__main__':
    main()