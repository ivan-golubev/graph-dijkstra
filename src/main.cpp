#include <array>
#include <vector>
#include <cassert>
#include <limits>
#include <queue>
#include <iostream>
#include <format>

struct Edge {
	int to;
	int weight;
};	

template<int VertexCount>
struct Graph {
public:
	std::array<std::vector<Edge>, VertexCount> AdjList;

	void AddEdge(int from, int to, int weight) {
		--from; --to; // 0 indexed
		assert(from >= 0 && from < VertexCount);
		assert(to >= 0 && to < VertexCount);
		AdjList[from].emplace_back(to, weight);
		AdjList[to].emplace_back(from, weight);
	}	
};

template<int VertexCount>
std::array<int, VertexCount> FindShortestPaths(Graph<VertexCount>& graph, int source) {
	--source;//0 indexed
	assert(source >= 0 && source < VertexCount);
	std::array<int, VertexCount> shortestPaths;
	shortestPaths.fill(std::numeric_limits<int>::max()); // distance to all vertices is unknown

	// Dijkstra's algorithm
	auto comparator = [](const Edge& left, const Edge& right) {
		return left.weight > right.weight;
	};
	std::priority_queue<Edge, std::vector<Edge>, decltype(comparator)> minHeap;
	for (const auto& edge: graph.AdjList[0])
		minHeap.push(edge);
	while (!minHeap.empty()) 
	{
		Edge shortest = minHeap.top(); minHeap.pop();
		std::cout << std::format("Edge from {} to {} with weight = {}\n", source+1, shortest.to+1, shortest.weight);

	}
	return shortestPaths;
}

int main() {
	Graph<6> graph;
	graph.AddEdge(1,2,7);
	graph.AddEdge(1,6,14);
	graph.AddEdge(1,3,9);

	graph.AddEdge(2,3,10);
	graph.AddEdge(2,4,15);

	graph.AddEdge(3,2,10);
	graph.AddEdge(3,6,2);
	graph.AddEdge(3,4,11);

	graph.AddEdge(4,5,6);
	graph.AddEdge(5,6,9);
	auto shortestPaths = FindShortestPaths(graph, 1);
	return 0;
}
