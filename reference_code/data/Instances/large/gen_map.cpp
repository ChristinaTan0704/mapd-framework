#include <iostream>
#include <cstdio>
#include <cstdlib>

using namespace std;

int map[111][111];
int main() {
	freopen("middle.map", "r", stdin);
	int row, col, endpoint, agent, maxtime;
	scanf("%d,%d", &row, &col);
	scanf("%d %d %d", &endpoint, &agent, &maxtime);
	int new_agent = 180;
	for (int i = 0; i < row; i++)
		for (int j = 0; j < col; j++) {
			char ch = ' ';
			while (ch != '.' && ch != '@' && ch != 'e' && ch != 'r')
				ch = getchar();
			if (ch == '.')
				map[i][j] = 0;
			else if (ch == '@')
				map[i][j] = 1;
			else if (ch == 'e')
				map[i][j] = 2;
			else
				map[i][j] = 3;
		}
	for (int i = 0; i < new_agent; i++) {
		int x = rand() % row, y = rand() % col;
		while (map[x][y] != 3)
			x = rand() % row, y = rand() % col;
		map[x][y] = 4;
	}
	freopen("kiva-180.map", "w", stdout);
	printf("%d,%d\n", row, col);
	printf("%d\n%d\n%d\n", endpoint, new_agent, maxtime);
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++)
			if (map[i][j] == 0 || map[i][j] == 3) printf("%c", '.');
			else if (map[i][j] == 1) printf("%c", '@');
			else if (map[i][j] == 4) printf("%c", 'r');
			else printf("%c", 'e');
		printf("\n");
	}
	return 0;
}