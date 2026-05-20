#include "Simulation.h"

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: cobra <map_file> <task_file>" << endl;
        return 1;
    }

    Simulation simu1(argv[1], argv[2]);
    simu1.run_TOTP();
    simu1.FullCollisionCheck("TP");
    simu1.SaveTask("tp_output.txt", argv[2]);

    Simulation simu2(argv[1], argv[2]);
    simu2.run_TPTR();
    simu2.FullCollisionCheck("TPTS");
    simu2.SaveTask("tptr_output.txt", argv[2]);

    cout << endl;
    simu1.ShowTask();
    simu2.ShowTask();
    return 0;
}
