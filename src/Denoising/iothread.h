#ifndef IOTHREAD_H
#define IOTHREAD_H

#include <QThread>
#include <QFileDialog>
#include "datamanager.h"

class ioThread : public QThread
{
    Q_OBJECT
public:
    ioThread(DataManager *_data_manager);
    ~ioThread();

    enum ioType{kNon, kImport, kExport};

signals:
    void statusShowMessage(QString);
    void setActionAndWidget(bool, bool);
    void needToUpdateGL(bool);

public:
    void setFileName(const QString file_name) {file_name_ = file_name;}
    void ImportMesh(QString &file_name);
    void ExportMesh(QString &file_name);

public:
    void run();

public:
    QString file_name_;
    ioType io_type_;
    QWidget *widget_;
    DataManager *data_manager_;
};

#endif // IOTHREAD_H
