#include <array>
#include <vector>
#include <cassert>
#include <limits>
#include <queue>
#include <iostream>
#include <format>

struct Edge
{
	int to;
	int weight;
};

struct Path
{
	int to;
	int distance;
};

template<int VertexCount>
struct Graph
{
public:
	std::array<std::vector<Edge>, VertexCount> AdjList;

	void AddEdge(int from, int to, int weight)
	{
		--from; --to; // 0 indexed
		assert(from >= 0 && from < VertexCount);
		assert(to >= 0 && to < VertexCount);
		AdjList[from].emplace_back(to, weight);
		AdjList[to].emplace_back(from, weight);
	}
};

template<int VertexCount>
std::array<int, VertexCount> FindShortestPaths(Graph<VertexCount>& graph, int sourceVertex)
{
	--sourceVertex; //0 indexed
	assert(sourceVertex >= 0 && sourceVertex < VertexCount);
	std::array<int, VertexCount> shortestPaths;
	shortestPaths.fill(std::numeric_limits<int>::max()); // distance to all vertices is unknown
	shortestPaths[sourceVertex] = 0; // path to the source vertex itself it zero
	// Dijkstra's algorithm
	auto comparator = [](const Path& left, const Path& right)
		{
			return left.distance > right.distance;
		};
	std::priority_queue<Path, std::vector<Path>, decltype(comparator)> minHeap;
	minHeap.emplace(sourceVertex, 0); // start at the source vertex
	while (!minHeap.empty())
	{
		auto [fromVertex, currentDistance] = minHeap.top(); minHeap.pop();
		for (const auto& [to, weight] : graph.AdjList[fromVertex])
		{
			int newDistance = weight + currentDistance;
			if (newDistance < shortestPaths[to])
			{
				shortestPaths[to] = newDistance;
#ifndef NDEBUG
				std::cout << std::format("Found shortest path from {} to {} with weight = {}\n", sourceVertex + 1, to + 1, newDistance);
#endif
				minHeap.emplace(to, newDistance);
			}
		}
	}
	return shortestPaths;
}

template<std::size_t VertexCount>
void printPaths(int source, const std::array<int, VertexCount>& paths)
{
	std::cout << "Shortest paths:\n";
	for (auto i = 0; i < paths.size(); ++i)
	{
		if (paths[i] < std::numeric_limits<int>::max())
			std::cout << std::format("{}->{} = {}\n", source, i + 1, paths[i]);
	}
}

int main()
{
	Graph<6> graph;
	graph.AddEdge(1, 2, 7);
	graph.AddEdge(1, 6, 14);
	graph.AddEdge(1, 3, 9);

	graph.AddEdge(2, 3, 10);
	graph.AddEdge(2, 4, 15);

	graph.AddEdge(3, 6, 2);
	graph.AddEdge(3, 4, 11);

	graph.AddEdge(4, 5, 6);
	graph.AddEdge(5, 6, 9);
	constexpr int from = 1;
	auto shortestPaths = FindShortestPaths(graph, from);
	printPaths(from, shortestPaths);
	return 0;
}
