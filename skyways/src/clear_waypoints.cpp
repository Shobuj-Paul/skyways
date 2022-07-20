#include <fstream>

using namespace std;

int main()
{
    string home = getenv("HOME");
    ofstream ClearLog(home + "/colcon_ws/src/skyways/files/WaypointLog.txt", ofstream::out | ofstream::trunc);
    ClearLog.close();
}