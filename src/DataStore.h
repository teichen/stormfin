// DataStore.h
#ifndef _DATASTORE
#define _DATASTORE
#include <cstring>
#include <vector>
#include <string>

using namespace std;

// static schema
struct DataRow {
    double* all_data;

    DataRow(double* z, double* u, double* x, size_t n_m, size_t n_u, size_t n_s) {
        double* all_data = new double[(n_m + n_u + n_s) / sizeof(double)];
        std::memcpy(all_data, z, n_m);
        std::memcpy((char*)all_data + n_m, u, n_u);
        std::memcpy((char*)all_data + n_u, x, n_s);
    }

    ~DataRow() {
        delete[] all_data;
    }
};

class DataStore
{
public:

    DataStore();

    std::vector<DataRow> buffer;
    void event_store(double*, double*, double*);
    std::vector<std::string> csv_header;

    ~DataStore();

private:
};

#endif
