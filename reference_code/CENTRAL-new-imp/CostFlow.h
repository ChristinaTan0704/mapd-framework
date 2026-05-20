#pragma once
#include <vector>
#include <queue>
#define INF 1111111111

struct Edge {
	int from, to, capcity, cost, loc, next, origin_capcity;
};

class CostFlow {
public:
	CostFlow(int node_cnt, int source, int sink);
	~CostFlow();
	void AddEdges(int from, int to, int capcity, int cost, int loc);
	void RemoveEdges(int from, int to);
	int MinCostFlow();
	std::vector<std::vector<int> > GetPath();
	int cost;
private:
	std::queue<int> Q;
	std::vector<Edge> edges;
	std::vector<int> head;
	std::vector<int> pre;
	std::vector<bool> in_queue;
	std::vector<int> dis;
	int node_cnt, source, sink;
	void AddEdge(int from, int to, int capcity, int cost, int loc);
	bool SPFA();
};