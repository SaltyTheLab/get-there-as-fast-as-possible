package org.example;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

public class Dijkstra {

    // Define a class to represent an edge with a destination node and weight
    static class Edge {
        int dest, weight;

        Edge(int dest, int weight) {
            this.dest = dest;
            this.weight = weight;
        }
    }

    // Method to find the shortest path using Dijkstra's algorithm
    public static List<Integer> dijkstra(List<List<Edge>> graph, int start, int end) {
        int n = graph.size();
        int[] dist = new int[n];
        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[start] = 0;

        int[] prev = new int[n];
        Arrays.fill(prev, -1);

        PriorityQueue<int[]> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a[1]));
        pq.add(new int[]{start, 0});

        while (!pq.isEmpty()) {
            int[] current = pq.poll();
            int u = current[0];
            int d = current[1];

            if (d > dist[u]) continue;

            for (Edge edge : graph.get(u)) {
                int v = edge.dest;
                int weight = edge.weight;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                    pq.add(new int[]{v, dist[v]});
                }
            }
        }

        // Reconstruct the shortest path
        List<Integer> path = new ArrayList<>();
        for (int at = end; at != -1; at = prev[at]) {
            path.add(at);
        }
        Collections.reverse(path);

        // Return the path if it exists, otherwise return -1
        return path.size() == 1 ? Collections.singletonList(-1) : path;
    }

    public static void main(String[] args) {
        // Create a graph with 7 nodes
        int n = 7;
        List<List<Edge>> graph = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            graph.add(new ArrayList<>());
        }

        // Add edges to the graph
        graph.get(0).add(new Edge(4, 2));
        graph.get(0).add(new Edge(3, 6));
        graph.get(1).add(new Edge(2, 5));
        graph.get(2).add(new Edge(4, 4));
        graph.get(2).add(new Edge(6, 8));
        graph.get(3).add(new Edge(5, 10));

        int start = 0;
        int end = 5;

        List<Integer> path = dijkstra(graph, start, end);

        if (path.get(0) == -1) {
            System.out.println("No path exists between the nodes.");
        } else {
            System.out.println("Shortest path: " + path);
            System.out.println("Distance: " + path.size());
        }
    }
}
