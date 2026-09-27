import math

class RFModel:
    def __init__(self, gcs_pos=(0, -100, 0), max_range=150.0):
        self.gcs_pos = gcs_pos
        self.max_range = max_range

    def get_pdr(self, dist):
        if dist <= 100.0:
            return 1.0
        elif dist >= self.max_range:
            return 0.0
        else:
            # Linear drop from 1.0 to 0.0 between 100m and 150m
            return 1.0 - ((dist - 100.0) / (self.max_range - 100.0))

    def compute_topology(self, active_drones):
        """
        active_drones: list of dicts with 'id', 'x', 'y', 'z'
        Returns:
            topology_data: dict mapping drone_id -> {
                'pdr': float (overall path reliability),
                'rssi': int (derived from pdr),
                'hops': list of node IDs in path (e.g. [0, 1, 3] where 0 is GCS),
                'parent_id': int (the previous node in the hop chain, 0 for GCS)
            }
        """
        nodes = {}
        nodes[0] = self.gcs_pos
        for d in active_drones:
            nodes[d['id']] = (d['x'], d['y'], d['z'])

        # Build adjacency matrix of negative log probabilities
        # We want to maximize reliability, which is prod(PDR_edges)
        # log(prod) = sum(log), so we minimize sum(-log(PDR))
        adj = {}
        for i in nodes:
            adj[i] = {}
            for j in nodes:
                if i != j:
                    dx = nodes[i][0] - nodes[j][0]
                    dy = nodes[i][1] - nodes[j][1]
                    dz = nodes[i][2] - nodes[j][2]
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                    pdr = self.get_pdr(dist)
                    if pdr > 0.01:
                        weight = -math.log(pdr)
                    else:
                        weight = float('inf')
                    adj[i][j] = {'weight': weight, 'pdr': pdr, 'dist': dist}

        # Dijkstra from GCS (node 0)
        dist_to = {n: float('inf') for n in nodes}
        dist_to[0] = 0.0
        parent = {n: None for n in nodes}
        unvisited = set(nodes.keys())

        while unvisited:
            curr = min(unvisited, key=lambda n: dist_to[n])
            if dist_to[curr] == float('inf'):
                break
            unvisited.remove(curr)

            for neighbor in unvisited:
                w = adj[curr][neighbor]['weight']
                new_dist = dist_to[curr] + w
                if new_dist < dist_to[neighbor]:
                    dist_to[neighbor] = new_dist
                    parent[neighbor] = curr

        topology = {}
        for d in active_drones:
            d_id = d['id']
            if dist_to[d_id] == float('inf'):
                topology[d_id] = {
                    'pdr': 0.0,
                    'rssi': 0,
                    'hops': [],
                    'parent_id': None
                }
            else:
                # Reconstruct path
                path = []
                curr = d_id
                while curr is not None:
                    path.insert(0, curr)
                    curr = parent[curr]
                
                overall_pdr = math.exp(-dist_to[d_id])
                rssi = int(30 + (overall_pdr * 68)) # Map 0-1 to 30-98 RSSI
                
                topology[d_id] = {
                    'pdr': overall_pdr,
                    'rssi': rssi,
                    'hops': path,
                    'parent_id': parent[d_id]
                }
                
        return topology
