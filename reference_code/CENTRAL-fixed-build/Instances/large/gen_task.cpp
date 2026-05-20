#include <cstdio>
#include <cstdlib>
using namespace std;

int main() {
	freopen("kiva-2000.task", "w", stdout);
	int endpoint = 480;
	int task = 2000;
	printf("%d\n", task);
	for (int i = 0; i < task; i++) {
		int u = rand() % endpoint, v = rand() % endpoint;
		while (u == v) 
			u = rand() % endpoint, v = rand() % endpoint;
		printf("0 %d %d 0 0\n", u, v);
	}
	return 0;
}