#include <iostream>
#include "Controller.h"
#include <gperftools/profiler.h>

using namespace std;

int main()
{
    ProfilerStart("/tmp/prof.out"); // memory profiler

    ProfilerStop();

    return 0;
}
