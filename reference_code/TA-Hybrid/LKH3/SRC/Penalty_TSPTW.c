#include "LKH.h"
#include "Segment.h"

int vis_agent[2000][100];
int vis_time[2000][100];

GainType Penalty_TSPTW()
{
    for (int i = 1; i <= 1000; i++) {
        vis_agent[i][0] = 0;
        vis_time[i][0] = 0;
    }

    Node *N = Depot, *NextN;
    GainType Sum = 0, P = 0, tmax = 0;
    int Forward = SUCC(N)->Id != N->Id + DimensionSaved;
    int now_agent = 0;
    //return 0;
    do {
        if (N->Id <= DimensionSaved) {
            if (N->Earliest < 0) {
                tmax = Sum > tmax ? Sum : tmax;
                if (tmax > CurrentPenalty)
                    return CurrentPenalty + 1;
                Sum = 0;
                now_agent++;
            }
            else {
                if (Sum < N->Earliest)
                    Sum = N->Earliest;
            }
            NextN = Forward ? SUCC(N) : PREDD(N);
        }
        NextN = Forward ? SUCC(N) : PREDD(N);
        Sum += (C(N, NextN) - N->Pi - NextN->Pi) / Precision;
    } while ((N = NextN) != Depot);
    tmax = Sum > tmax ? Sum : tmax;
    return tmax;
}
