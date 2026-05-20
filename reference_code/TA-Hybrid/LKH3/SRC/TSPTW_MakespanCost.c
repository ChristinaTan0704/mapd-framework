#include "LKH.h"
#include "Segment.h"

GainType TSPTW_MakespanCost()
{
    Node *N = Depot, *NextN;
    GainType Sum = 0, t = 0;
    int Forward = SUCC(N)->Id != N->Id + DimensionSaved;
    if (ProblemType != TSPTW)
        return 0;
    do {
        if (N->Id <= DimensionSaved) {
            if (N->Earliest < 0) {
                Sum += t;
                t = 0;
            }
            if (t < N->Earliest) {
                t = N->Earliest;
            }
        }
        NextN = Forward ? SUCC(N) : PREDD(N);
        t += (C(N, NextN) - N->Pi - NextN->Pi) / Precision;
    } while ((N = NextN) != Depot);
    Sum += t;
    return Sum; 
}
