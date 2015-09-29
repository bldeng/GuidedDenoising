#include "meshdenoisebase.h"

MeshDenoiseBase::MeshDenoiseBase(DataManager *_data_manager, ParameterSet *_parameter_set)
{
    if((_data_manager == NULL) || (_parameter_set == NULL))
        return;

    data_manager_ = _data_manager;
    parameter_set_ = _parameter_set;
}
