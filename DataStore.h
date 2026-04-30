// DataStore.h
#ifndef _DATASTORE
#define _DATASTORE

using namespace std;

// static schema
struct DataRow {
    double z0;
    double z1;
    double u0;
    double u1;
    double x0;
    double x1;
};

class DataStore
{
public:

    DataStore();

    std::vector<DataRow> buffer;
    void event_store(double*, double*, double*);

    ~DataStore();

private:
};

#endif
