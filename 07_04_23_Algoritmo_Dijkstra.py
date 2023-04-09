import heapq
import networkx as nx
import matplotlib.pyplot as plt
import random 

def dijkstra(G, start):
    distances = {node: float('inf') for node in G.nodes()}
    distances[start] = 0
    parents = {node: None for node in G}
    pq = [(0, start)]
    while len(pq) > 0:
        (dist, current) = heapq.heappop(pq)
        if dist > distances[current]:
            continue
        for neighbor in G.neighbors(current):
            distance = dist + G.edges[current, neighbor]['weight']
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parents[neighbor] = current
                heapq.heappush(pq, (distance, neighbor))
    return distances, parents

# Se cra el grafo, primero se agregan los nodos y luego se agrega sus vertices, luego con nuestros random se le agregan valores a cada uno de nuestros vertices
vertices = [random.randint(1,10)for _ in range(15)]
G = nx.Graph()
G.add_nodes_from(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'])
G.add_edge('A', 'B', weight=vertices[0])
G.add_edge('A', 'C', weight=vertices[1])
G.add_edge('A', 'D', weight=vertices[2])
G.add_edge('B', 'C', weight=vertices[3])
G.add_edge('B', 'F', weight=vertices[4])
G.add_edge('B', 'H', weight=vertices[5])
G.add_edge('C', 'D', weight=vertices[6])
G.add_edge('C', 'E', weight=vertices[7])
G.add_edge('C', 'H', weight=vertices[8])
G.add_edge('D', 'E', weight=vertices[9])
G.add_edge('E', 'H', weight=vertices[10])
G.add_edge('E', 'G', weight=vertices[11])
G.add_edge('F', 'H', weight=vertices[12])
G.add_edge('F', 'G', weight=vertices[13])
G.add_edge('H', 'G', weight=vertices[14])

distances, parents = dijkstra(G, 'A')

# Se imprime la distancia minima entra cada de sus nodos y sus caminos de como se hace
print("Distancias mínimas desde el nodo de inicio:")
for node, dist in distances.items():
    print(f"{node}: {dist}")
    
print("Caminos más cortos:")
for node, parent in parents.items():
    path = []
    while parent is not None:
        path.append(parent)
        parent = parents[parent]
    path.reverse()
    path.append(node)
    print(" -> ".join(path))

# Se crea de forma grafica nuestro grafo que previamente se realizo
pos = nx.spring_layout(G)
nx.draw_networkx_nodes(G, pos)
nx.draw_networkx_edges(G, pos)
nx.draw_networkx_labels(G, pos)
nx.draw_networkx_edge_labels(G, pos, edge_labels={(u, v): d['weight'] for u, v, d in G.edges(data=True)})
plt.show()