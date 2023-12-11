import numpy as np
import networkx as nx
from atom_core.naming import generateKey
from atom_core.geometry import translationQuaternionToTransform


def getChain(from_frame, to_frame, transform_pool):
    """ Gets a chain of transforms given two reference frames and a se of transformations. Computes a graph from the
    set of transforms, and then finds a path in the graph betweem the two given links.

    @param from_frame: initial frame
    @param to_frame: final frame
    @param transform_pool: a dictionary containing several transforms
    @return:  a chain of transforms The standard we have is to use a list of dictionaries, each containing
    # information about the transform: [{'parent': parent, 'child': child, 'key': 'parent-child'}, {...}]
    """
    chain = []  # initialized to empty list. The standard we have is to use a list of dictionaries, each containing
    # information about the transform: [{'parent': parent, 'child': child, 'key': 'parent-child'}, {...}]

    graph = nx.Graph()  # build a graph of transforms and then use it to find the path
    for transform_key, transform in transform_pool.items():  # create the graph from the transform_pool
        graph.add_edge(transform['parent'], transform['child'])

    # Debug stuff, just for drawing
    # nx.draw(graph, with_labels=True)
    # import matplotlib.pyplot as plt
    # plt.show()

    path = nx.shortest_path(graph, from_frame, to_frame)  # compute the path between given reference frames
    for idx in range(0, len(path) - 1):  # get the chain as a list of dictionaries from the path
        parent = path[idx]
        child = path[idx + 1]
        chain.append({'parent': parent, 'child': child, 'key': generateKey(parent, child)})

    return chain


def getAggregateTransform(chain, transforms):
    """ Multiplies local transforms in a chain to get the global transform of the chain

    @param chain: a list of transforms
    @param transforms: a pool of transformations
    @return: the global transformation (4x4 homogeneous)
    """
    transform = np.eye(4, dtype=float)

    for link in chain:

        key = generateKey(link['parent'], link['child'])
        inverse_key = generateKey(link['child'], link['parent'])
        if key in transforms.keys():  # check if link exists in transforms
            trans = transforms[key]['trans']
            quat = transforms[key]['quat']
            parent_T_child = translationQuaternionToTransform(trans, quat)
            # print(parent + '_T_' + child + ' =\n' + str(parent_T_child))
        elif inverse_key in transforms.keys():  # the reverse transform may exist
            trans = transforms[inverse_key]['trans']
            quat = transforms[inverse_key]['quat']
            parent_T_child = np.linalg.inv(translationQuaternionToTransform(trans, quat))
        else:
            raise ValueError('Transform from ' + link['parent'] + ' to ' + link['child'] + ' does not exist.')

        transform = np.dot(transform, parent_T_child)
    # print(parent + '_T_' + child + ' =\n' + str(AT))

    return transform


def getTransform(from_frame, to_frame, transforms):
    """ Gets a transformation between any two frames

    @param from_frame: Starting frame
    @param to_frame: Ending frame
    @param transforms: dictionary of several transforms
    @return: the global transformation (4x4 homogeneous)
    """
    chain = getChain(from_frame, to_frame, transforms)
    return getAggregateTransform(chain, transforms)
