#include <array>
#include <vector>
#include <cassert>
#include <limits>
#include <queue>
#include <iostream>
#include <format>

struct Edge {
	int from;
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
		AdjList[from].emplace_back(from, to, weight);
		AdjList[to].emplace_back(to, from, weight);
	}	
};

template<int VertexCount>
std::array<int, VertexCount> FindShortestPaths(Graph<VertexCount>& graph, int source) {
	--source;//0 indexed
	assert(source >= 0 && source < VertexCount);
	std::array<int, VertexCount> shortestPaths;
	shortestPaths.fill(std::numeric_limits<int>::max()); // distance to all vertices is unknown
	shortestPaths[source] = 0; // path to the source itself
	// Dijkstra's algorithm
	auto comparator = [](const Edge& left, const Edge& right) {
		return left.weight > right.weight;
	};
	std::priority_queue<Edge, std::vector<Edge>, decltype(comparator)> minHeap;
	for (const auto& edge: graph.AdjList[source])
		minHeap.push(edge);
	std::array<bool,VertexCount> visitedVertices = {};
	while (!minHeap.empty()) 
	{
		Edge edge = minHeap.top(); minHeap.pop();
		shortestPaths[edge.to] = std::min(shortestPaths[edge.to], shortestPaths[edge.from]+edge.weight);
#ifndef NDEBUG
		std::cout << std::format("Edge from {} to {} with weight = {}\n", source+1, edge.to+1, shortestPaths[edge.to]);
#endif
		visitedVertices[edge.from] = true;
		// push the edges from this vertex
		for (const auto& edge: graph.AdjList[edge.to]) {
			if (!visitedVertices[edge.to])
				minHeap.push(edge);
		}
	}
	return shortestPaths;
}

template<std::size_t VertexCount>
void printPaths(int source, const std::array<int, VertexCount>& paths) {
	std::cout << "Shortest paths:\n";
	for (auto i=0; i<paths.size(); ++i) {
		if (paths[i] < std::numeric_limits<int>::max())
			std::cout << std::format("{}->{} = {}\n", source, i+1, paths[i]);
	}
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
	constexpr int from = 1;
	auto shortestPaths = FindShortestPaths(graph, from);
	printPaths(from, shortestPaths);
	return 0;
}
