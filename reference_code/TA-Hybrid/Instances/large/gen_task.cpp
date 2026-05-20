#include <cstdio>
#include <cstdlib>
#include <algorithm>
using namespace std;

int main() {
	freopen("kiva-500-varying.task", "w", stdout);
	int endpoint = 302;
	int task = 500;
	printf("%d\n", task);
	for (int i = 0; i < task; i++) {
		int num = (rand() % 5 + 1);
		int arr[5];
		for (int j = 0; j < num; j++)
		{
			int u = rand() % endpoint;
			while (std::find(std::begin(arr), std::end(arr), u) != std::end(arr)) 
				u = rand() % endpoint;
			arr[j] = u;
		}
		printf("%d ", 0);
		for (int i = 0; i < num; i++)
		{
			printf("%d ", arr[i]);
		}
		printf("\n");
	}
	return 0;
}