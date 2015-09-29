#include "iothread.h"

ioThread::ioThread(DataManager *_data_manager)
{
    if(_data_manager != NULL)
        data_manager_ = _data_manager;
}

ioThread::~ioThread()
{

}

void ioThread::ImportMesh(QString &file_name)
{
    emit(setActionAndWidget(false, false));
    emit(statusShowMessage("Now loading mesh " + file_name + " ..."));
    if(!data_manager_->ImportMeshFromFile(file_name.toStdString()))
    {
        emit(statusShowMessage("Loading mesh " + file_name + " failed."));
        return;
    }
    else
        emit(statusShowMessage("Loading mesh " + file_name + " successful."));

    emit(needToUpdateGL(true));
    emit(setActionAndWidget(true, true));
}

void ioThread::ExportMesh(QString &file_name)
{
    emit(statusShowMessage("Now writing mesh " + file_name + " ..."));
    if(!data_manager_->ExportMeshToFile(file_name.toStdString()))
    {
        emit(statusShowMessage("Writing mesh " + file_name + " failed."));
        return;
    }
    else
        emit(statusShowMessage("Writing mesh " + file_name + " successful."));
}

void ioThread::run()
{
    if(io_type_ == kImport)
        ImportMesh(file_name_);
    else
        ExportMesh(file_name_);
}
