#include "parametersetwidget.h"
#include <iostream>

ParameterWidget::ParameterWidget(QWidget *parent, Parameter *_parameter)
    : QWidget(parent)
{
    if(_parameter == NULL)
        return;

    parameter_ = _parameter;

//    QVBoxLayout *VBoxLayout = new QVBoxLayout;
//    this->setLayout(VBoxLayout);
    label_ = new QLabel(parameter_->getLabel());
    this->setToolTip(parameter_->getToolTip());
    this->setWhatsThis(parameter_->getToolTip());
//    VBoxLayout->addWidget(label_);
    if(parameter_->getType() == Parameter::kBool){
        QCheckBox *CheckBox = new QCheckBox;
//        VBoxLayout->addWidget(CheckBox);
        CheckBox->setChecked(parameter_->getValueBool());
        CheckBox->setText(parameter_->getLabel());
        widget_ = CheckBox;
        widget_type_ = kCheckBox;
    }
    else if(parameter_->getType() == Parameter::kInt){
        QLineEdit *LineEdit = new QLineEdit;
        if(parameter_->getHasValidator()){
            int minvalue = parameter_->getValidMin().toInt();
            int maxvalue = parameter_->getValidMax().toInt();
            LineEdit->setValidator(new QIntValidator(minvalue,maxvalue,this));
        }
        LineEdit->setText(QString::number(parameter_->getValueInt()));
//        VBoxLayout->addWidget(LineEdit);
        widget_ = LineEdit;
        widget_type_ = kLineEdit;
    }
    else if(parameter_->getType() == Parameter::kDouble){
        QLineEdit *LineEdit = new QLineEdit;
        if(parameter_->getHasValidator()){
            double minvalue = parameter_->getValidMin().toDouble();
            double maxvalue = parameter_->getValidMax().toDouble();
            LineEdit->setValidator(new QDoubleValidator(minvalue,maxvalue,9,this));
        }
        LineEdit->setText(QString::number(parameter_->getValueDouble()));
//        VBoxLayout->addWidget(LineEdit);
        widget_ = LineEdit;
        widget_type_ = kLineEdit;
    }
    else if(parameter_->getType() == Parameter::kQStringList){
        QComboBox *ComboBox = new QComboBox;
        ComboBox->addItems(parameter_->getValueQStringList());
        ComboBox->setCurrentIndex(parameter_->getStringListIndex());
//        VBoxLayout->addWidget(ComboBox);
        widget_ = ComboBox;
        widget_type_ = kComboBox;
    }
}

void ParameterWidget::setParameter()
{
    if(parameter_->getType() ==  Parameter::kInt)
    {
        QLineEdit* LineEdit = dynamic_cast<QLineEdit *>(widget_);
        LineEdit->setText(QString::number(parameter_->getValueInt()));
    }
    else if(parameter_->getType() == Parameter::kDouble)
    {
        QLineEdit* LineEdit = dynamic_cast<QLineEdit *>(widget_);
        LineEdit->setText(QString::number(parameter_->getValueDouble()));
    }
    else if(parameter_->getType() == Parameter::kBool)
    {
         QCheckBox *pCheckBox =  dynamic_cast<QCheckBox *>(widget_);
         pCheckBox->setChecked(parameter_->getValueBool());
    }
    else
    {
         QComboBox *ComboBox =  dynamic_cast<QComboBox *>(widget_);
         ComboBox->setCurrentIndex(parameter_->getStringListIndex());
    }
    update();
}

void ParameterWidget::resetParameter()
{
    if(parameter_->getType() ==  Parameter::kInt)
    {
        QLineEdit* LineEdit = dynamic_cast<QLineEdit *>(widget_);
        LineEdit->setText(QString::number(parameter_->getDefaultValue().toInt()));
    }
    else if(parameter_->getType() == Parameter::kDouble)
    {
        QLineEdit* LineEdit = dynamic_cast<QLineEdit *>(widget_);
        LineEdit->setText(QString::number(parameter_->getDefaultValue().toDouble()));
    }
    else if(parameter_->getType() == Parameter::kBool)
    {
         QCheckBox *pCheckBox =  dynamic_cast<QCheckBox *>(widget_);
         pCheckBox->setChecked(parameter_->getDefaultValue().toBool());
    }
    else
    {
         QComboBox *ComboBox =  dynamic_cast<QComboBox *>(widget_);
         ComboBox->setCurrentIndex(parameter_->getDefaultIndex());
    }
    update();
}

void ParameterWidget::updateParameter()
{
    if(parameter_->getType() ==  Parameter::kInt)
    {
        QLineEdit* LineEdit = dynamic_cast<QLineEdit *>(widget_);
        QString t = LineEdit->text();
        bool ok;
        int value = t.toInt(&ok);
        if(ok)
        {
            parameter_->setValue(value);
        }
    }
    else if(parameter_->getType() == Parameter::kDouble)
    {
        QLineEdit* LineEdit = dynamic_cast<QLineEdit *>(widget_);
        QString t = LineEdit->text();
        bool ok;
        double value = t.toDouble(&ok);
        if(ok)
        {
            parameter_->setValue(value);
        }
    }
    else if(parameter_->getType() == Parameter::kBool)
    {
         QCheckBox *CheckBox =  dynamic_cast<QCheckBox *>(widget_);
         parameter_->setValue(CheckBox->isChecked());
    }
    else if(parameter_->getType() == Parameter::kQStringList)
    {
         QComboBox *ComboBox =  dynamic_cast<QComboBox *>(widget_);
         parameter_->setStringListIndex(ComboBox->currentIndex());
    }
    update();
}


ParameterSetWidget::ParameterSetWidget(QWidget *parent, ParameterSet *_parameter_set)
    : QDockWidget(parent)
{
    if(_parameter_set == NULL)
        return;

    parameter_set_ = _parameter_set;
    parameter_widget_list_.clear();

    Frame_ = new QFrame;
    setWidget(Frame_);

    this->setStyleSheet("font-size:14px;");
    VBoxLayout_ = new QVBoxLayout;
    Frame_->setLayout(VBoxLayout_);
    Label_ = new QLabel;
    Label_->setWordWrap(true);
    VBoxLayout_->addWidget(Label_);

    QPushButton *apply = new QPushButton(tr("Apply"), this);
    connect(apply, SIGNAL(clicked()), this, SLOT(OnApplyClick()));
    VBoxLayout_->addWidget(apply);

    this->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    this->hide();
}

void ParameterSetWidget::OnDefaultClick()
{
    for(int i = 0; i < (int)parameter_widget_list_.size(); i++)
        parameter_widget_list_[i]->resetParameter();
}

void ParameterSetWidget::OnApplyClick()
{
    for(int i = 0; i < (int)parameter_widget_list_.size(); i++)
        parameter_widget_list_[i]->updateParameter();

    emit(ReadyToApply(parameter_set_->getName()));
}

void ParameterSetWidget::showWidget()
{
    this->setWindowTitle(parameter_set_->getLabel());
    Label_->setText(parameter_set_->getIntroduction());

    QGroupBox *GroupBox = new QGroupBox("Setting");
    VBoxLayout_Setting_ = new QVBoxLayout(GroupBox);
    QVector<Parameter *> all_parameters = parameter_set_->getAllParameters();
    for(int i = 0; i < (int)all_parameters.size(); i++){
        Parameter *temp_parameter = all_parameters[i];
        ParameterWidget *parameter_widget = new ParameterWidget(this, temp_parameter);
        parameter_widget_list_.append(parameter_widget);
//        VBoxLayout_Setting_->addWidget(parameter_widget);
        if(parameter_widget->widget_type_ != ParameterWidget::kCheckBox)
            VBoxLayout_Setting_->addWidget(parameter_widget->getLabel());
        VBoxLayout_Setting_->addWidget(parameter_widget->getWidget());
    }

    VBoxLayout_Setting_->setAlignment(Qt::AlignTop);
    VBoxLayout_->insertWidget(1, GroupBox);
    VBoxLayout_->setAlignment(Qt::AlignTop);
    Frame_->showNormal();
    Frame_->adjustSize();
    this->showNormal();
    this->adjustSize();
}
