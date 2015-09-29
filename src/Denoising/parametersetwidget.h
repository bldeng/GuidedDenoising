#ifndef PARAMETERWIDGET_H
#define PARAMETERWIDGET_H

#include "parameterset.h"
#include <QWidget>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QGridLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QGroupBox>

class ParameterWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ParameterWidget(QWidget *parent, Parameter *_parameter);

    enum WidgetType{kNon, kCheckBox, kLineEdit, kComboBox};

    void setParameter();
    void resetParameter();
    void updateParameter();

    QLabel *getLabel() const {return label_;}
    QWidget *getWidget() const {return widget_;}

    WidgetType widget_type_;
private:
    Parameter *parameter_;
    QLabel *label_;
    QWidget *widget_;
};

class ParameterSetWidget : public QDockWidget
{
    Q_OBJECT
public:
    explicit ParameterSetWidget(QWidget *parent, ParameterSet *_parameter_set);

signals:
    void ReadyToApply(QString _algorithms_name);
    void TransToNoisyMesh();
    void TransToOriginalMesh();
    void TransToDenoisedMesh();

public slots:
    void OnDefaultClick();
    void OnApplyClick();

public:
    void showWidget();

private:
    QFrame *Frame_;
    ParameterSet *parameter_set_;
    QVBoxLayout *VBoxLayout_;
    QVBoxLayout *VBoxLayout_Setting_;
    QLabel *Label_;
    QList<ParameterWidget *> parameter_widget_list_;
};

#endif // PARAMETERWIDGET_H
