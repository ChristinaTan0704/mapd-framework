#include <cstdio>
#include <cstdlib>
using namespace std;

int main() {
	// freopen("kiva-2000.task", "w", stdout);
	freopen("kiva-1.task", "w", stdout);
	// int endpoint = 480;
	// int task = 2000;
	int endpoint = 302;
	int task = 500;
	printf("%d\n", task);
	for (int i = 0; i < task; i++) {
		int u = rand() % endpoint, v = rand() % endpoint;
		while (u == v) 
			u = rand() % endpoint, v = rand() % endpoint;
		printf("%d %d %d 0 0\n", i, u, v);
	}
	return 0;
}