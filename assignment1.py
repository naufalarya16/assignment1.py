import heapq
import math
import time

def a_star_search(graph, distances, start, goal):
    def heuristic(node):
        x1, y1 = distances[node]
        x2, y2 = distances[goal]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    start_time = time.time()
    open_set = []
    closed_set = set()
    visited_count = 0
    heapq.heappush(open_set, (heuristic(start), start, [start], 0))

    while open_set:
        f, current, path, g_score = heapq.heappop(open_set)
        visited_count += 1

        if current == goal:
            time_taken = (time.time() - start_time) * 1000
            return path, visited_count, time_taken

        if current in closed_set:
            continue
        closed_set.add(current)

        for neighbor in graph.get(current, []):
            if neighbor in closed_set:
                continue

            x1, y1 = distances[current]
            x2, y2 = distances[neighbor]
            edge_cost = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            new_g_score = g_score + edge_cost
            new_f_score = new_g_score + heuristic(neighbor)
            heapq.heappush(open_set, (new_f_score, neighbor, path + [neighbor], new_g_score))

    time_taken = (time.time() - start_time) * 1000
    return None, visited_count, time_taken

def greedy_best_first_search(graph, distances, start, goal):
    def heuristic(node):
        x1, y1 = distances[node]
        x2, y2 = distances[goal]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    start_time = time.time()
    open_set = []
    closed_set = set()
    visited_count = 0
    heapq.heappush(open_set, (heuristic(start), start, [start]))

    while open_set:
        h, current, path = heapq.heappop(open_set)
        visited_count += 1

        if current == goal:
            time_taken = (time.time() - start_time) * 1000
            return path, visited_count, time_taken

        if current in closed_set:
            continue
        closed_set.add(current)

        for neighbor in graph.get(current, []):
            if neighbor in closed_set:
                continue
            heapq.heappush(open_set, (heuristic(neighbor), neighbor, path + [neighbor]))

    time_taken = (time.time() - start_time) * 1000
    return None, visited_count, time_taken

def visualize_map(cities, path, start, goal):
    max_x = max(x for x, y in cities.values()) + 1
    max_y = max(y for x, y in cities.values()) + 1
    grid = [['   ' for _ in range(max_x * 2 + 1)] for _ in range(max_y * 2 + 1)]

    # Tandai kota biasa
    for city, (x, y) in cities.items():
        if city not in path and city != start and city != goal:
            grid[y * 2][x * 2] = '[.]'

    # Tandai jalur
    for i in range(len(path)):
        city = path[i]
        x, y = cities[city]
        if city == start:
            symbol = '[S]'
        elif city == goal:
            symbol = '[G]'
        else:
            symbol = '[*]'
        grid[y * 2][x * 2] = symbol

    # Gambar garis penghubung antar kota dalam path
    for i in range(len(path) - 1):
        city1, city2 = path[i], path[i + 1]
        x1, y1 = cities[city1]
        x2, y2 = cities[city2]

        steps = max(abs(x2 - x1), abs(y2 - y1)) * 2
        for step in range(1, steps):
            x = int(x1 * 2 + (x2 - x1) * 2 * step / steps)
            y = int(y1 * 2 + (y2 - y1) * 2 * step / steps)
            if grid[y][x] == '   ':
                grid[y][x] = ' - '

    # Cetak sumbu X
    print("\n    " + ''.join([f"{i:^3}" for i in range(max_x * 2 + 1)]))
    for idx, row in enumerate(grid):
        print(f"{idx:^3} " + ''.join(row))

if __name__ == "__main__":
    # Data kota dan koneksi
    cities = {
        "A": (0, 0),
        "B": (2, 1),
        "C": (4, 2),
        "D": (5, 5),
        "E": (1, 4)
    }

    roads = {
        "A": ["B", "E"],
        "B": ["A", "C"],
        "C": ["B", "D"],
        "D": ["C"],
        "E": ["A", "D"]
    }

    start_city = "A"
    goal_city = "D"

    print(f"Mencari rute dari {start_city} ke {goal_city}:\n")

    # A* Search
    a_star_path, a_star_visited, a_star_time = a_star_search(roads, cities, start_city, goal_city)
    print("A* Search:")
    print(f"  Jalur: {' -> '.join(a_star_path)}")
    print(f"  Node dikunjungi: {a_star_visited}")
    print(f"  Waktu eksekusi: {a_star_time:.2f} ms")
    print("  Visualisasi:")
    visualize_map(cities, a_star_path, start_city, goal_city)

    # GBFS
    gbfs_path, gbfs_visited, gbfs_time = greedy_best_first_search(roads, cities, start_city, goal_city)
    print("\nGreedy Best-First Search:")
    print(f"  Jalur: {' -> '.join(gbfs_path)}")
    print(f"  Node dikunjungi: {gbfs_visited}")
    print(f"  Waktu eksekusi: {gbfs_time:.2f} ms")
    print("  Visualisasi:")
    visualize_map(cities, gbfs_path, start_city, goal_city)
