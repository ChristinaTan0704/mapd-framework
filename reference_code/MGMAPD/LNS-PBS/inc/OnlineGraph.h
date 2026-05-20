#pragma once
#include "BasicGraph.h"


class OnlineGrid :
	public BasicGraph
{
public:
	vector<int> entries;
	vector<int> exits;
    bool load_map(string fname);
	bool load_Minghua_map(string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
};
