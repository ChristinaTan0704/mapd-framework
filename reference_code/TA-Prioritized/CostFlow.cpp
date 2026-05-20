#include "CostFlow.h"

CostFlow::CostFlow(int node_cnt, int source, int sink): node_cnt(node_cnt), source(source), sink(sink) {
	//printf("node: %d\n", node_cnt);
	cost = 0;
	head.resize(node_cnt, -1);
	pre.resize(node_cnt, -1);
	in_queue.resize(node_cnt, false);
	dis.resize(node_cnt, INF);
	return ;
}

CostFlow::~CostFlow() {
}

void CostFlow::AddEdge(int from, int to, int capcity, int cost, int loc) {
	Edge e;
	e.from = from;
	e.to = to;
	e.origin_capcity = capcity;
	e.capcity = capcity; 
	e.cost = cost;
	e.loc = loc;
	e.next = head[from];
	edges.push_back(e);
	head[from] = edges.size() - 1;
	return ;
}

void CostFlow::AddEdges(int from, int to, int capcity, int cost, int loc) {
	AddEdge(from, to, capcity, cost, loc);
	AddEdge(to, from, 0, -cost, loc);
	return ;
}

void CostFlow::RemoveEdges(int from, int to) { // remove edges before cost flow
	for (int i = head[from]; i != -1; i = edges[i].next) 
		if (edges[i].to == to && edges[i].capcity > 0) {
			edges[i].capcity--;
			edges[i].origin_capcity--;
			return ;
		}
	return ;
}

bool CostFlow::SPFA() { // find shortest path from source
	while (!Q.empty()) Q.pop();
	for (int i = 0; i < node_cnt; i++) {
		dis[i] = INF;
		in_queue[i] = false;
	}
	dis[source] = 0;
	Q.push(source);
	in_queue[source] = true;
	while (!Q.empty()) {
		int u = Q.front();
		Q.pop();
		in_queue[u] = false;
		for (int i = head[u]; i != -1; i = edges[i].next) 
			if (edges[i].capcity > 0) {
				int v = edges[i].to, cost = edges[i].cost;
				int dd = dis[u] + cost;
				if (dis[v] > dd) {
					dis[v] = dd;
					pre[v] = i;
					if (!in_queue[v]) {
						Q.push(v);
						in_queue[v] = true;
					}
				}
			}
	}
	return dis[sink] < INF;
}

int CostFlow::MinCostFlow() { // for unit graph
	int flow = 0;
	while (SPFA()) {
		flow++;
		int u = sink;
		while (u != source) {
			edges[pre[u]].capcity--;
			edges[pre[u] ^ 1].capcity++;
			cost += edges[pre[u]].cost;
			u = edges[pre[u]].from;
		}
	}
	return flow;
}

std::vector<std::vector<int> > CostFlow::GetPath() {
	std::vector<std::vector<int> > paths;
	for (int i = head[source]; i != -1; i = edges[i].next) 
		if (edges[i].capcity < edges[i].origin_capcity) {
			std::vector<int> path;
			path.push_back(edges[i].loc);
			int u = edges[i].to;
			while (u != sink) {
				int v = -1;
				for (int j = head[u]; j != -1; j = edges[j].next) 
					if (edges[j].capcity < edges[j].origin_capcity) {
						v = edges[j].to;
						if (edges[j].loc != -1)
							path.push_back(edges[j].loc);
						u = v;
						break;
					}
				if (v == -1) {
					printf("%d GETPATH ERROR\n", cost);
					while (1);
				}
			}
			paths.push_back(path);
		}
	return paths;
}
