#ifndef MESHDENOISEBASE_H
#define MESHDENOISEBASE_H

#include "../datamanager.h"
#include "../parameterset.h"

class MeshDenoiseBase
{
public:
    MeshDenoiseBase(DataManager *_data_manager, ParameterSet *_parameter_set);
    virtual ~MeshDenoiseBase() {}

    DataManager *getDataManager() const {return data_manager_;}
    void setDataManager(DataManager *_data_manager) {data_manager_ = _data_manager;}
    ParameterSet *getParameterSet() const {return parameter_set_;}
    void setParameterSet(ParameterSet *_parameter_set) {parameter_set_ = _parameter_set;}

    virtual void denoise() = 0;

private:
    virtual void initParameters() = 0;

private:
    DataManager *data_manager_;
    ParameterSet *parameter_set_;
};

#endif // MESHDENOISEBASE_H
