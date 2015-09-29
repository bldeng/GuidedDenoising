#include "parameterset.h"

ParameterSet::ParameterSet()
{
    parameter_dictionary_.clear();
}

ParameterSet::~ParameterSet()
{
    parameter_dictionary_.clear();
}

bool ParameterSet::getValue(const QString &_name, QVariant &_value) const
{
    if(!parameter_dictionary_.contains(_name)){
        _value = QVariant();
        return false;
    }
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kQVariant){
            _value = QVariant();
            return false;
        }
        _value = p->getValueQVariant();
        return true;
    }
}

void ParameterSet::setValue(const QString &_name, const QVariant &_value)
{
    if(!parameter_dictionary_.contains(_name))
        return;
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kQVariant)
            return;
        p->setValue(_value);
    }
}

bool ParameterSet::getValue(const QString &_name, bool &_value) const
{
    if(!parameter_dictionary_.contains(_name)){
        _value = false;
        return false;
    }
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kBool){
            _value = false;
            return false;
        }
        _value = p->getValueBool();
        return true;
    }
}

void ParameterSet::setValue(const QString &_name, const bool &_value)
{
    if(!parameter_dictionary_.contains(_name))
        return;
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kBool)
            return;
        p->setValue(_value);
    }
}

bool ParameterSet::getValue(const QString &_name, int &_value) const
{
    if(!parameter_dictionary_.contains(_name)){
        _value = 0;
        return false;
    }
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kInt){
            _value = 0;
            return false;
        }
        _value = p->getValueInt();
        return true;
    }
}

void ParameterSet::setValue(const QString &_name, const int &_value)
{
    if(!parameter_dictionary_.contains(_name))
        return;
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kInt)
            return;
        p->setValue(_value);
    }
}

bool ParameterSet::getValue(const QString &_name, double &_value) const
{
    if(!parameter_dictionary_.contains(_name)){
        _value = 0.0;
        return false;
    }
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kDouble){
            _value = 0.0;
            return false;
        }
        _value = p->getValueDouble();
        return true;
    }
}

void ParameterSet::setValue(const QString &_name, const double &_value)
{
    if(!parameter_dictionary_.contains(_name))
        return;
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kDouble)
            return;
        p->setValue(_value);
    }
}

bool ParameterSet::getValue(const QString &_name, QStringList &_value) const
{
    if(!parameter_dictionary_.contains(_name)){
        _value = QStringList();
        return false;
    }
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kQStringList){
            _value = QStringList();
            return false;
        }
        _value = p->getValueQStringList();
        return true;
    }
}

void ParameterSet::setValue(const QString &_name, const QStringList &_value)
{
    if(!parameter_dictionary_.contains(_name))
        return;
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kQStringList)
            return;
        p->setValue(_value);
    }
}

bool ParameterSet::getStringListIndex(const QString &_name, int &_index) const
{
    if(!parameter_dictionary_.contains(_name)){
        _index = 0;
        return false;
    }
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kQStringList){
            _index = 0;
            return false;
        }
        _index = p->getStringListIndex();
        return true;
    }
}

void ParameterSet::setStringListIndex(const QString &_name, const int &_index)
{
    if(!parameter_dictionary_.contains(_name))
        return;
    else{
        Parameter *p = parameter_dictionary_.value(_name);
        if(p->getType() != Parameter::kQStringList)
            return;
        p->setStringListIndex(_index);
    }
}

void ParameterSet::addParameter(const QString &_name, bool _defaultValue, const QString &_label, const QString &_toolTip)
{
    if(!parameter_dictionary_.contains(_name)){
        Parameter *p = new Parameter(_defaultValue);
        addParameter(_name, p, _label, _toolTip);
    }
}

void ParameterSet::addParameter(const QString &_name, int _defaultValue, const QString &_label, const QString &_toolTip,
                                bool _hasValidator, int _validMin, int _validMax)
{
    if(!parameter_dictionary_.contains(_name)){
        Parameter *p = new Parameter(_defaultValue);
        if(_hasValidator){
            p->setHasValidator(_hasValidator);
            p->setValidMin((QVariant)_validMin);
            p->setValidMax((QVariant)_validMax);
        }
        addParameter(_name, p, _label, _toolTip);
    }
}

void ParameterSet::addParameter(const QString &_name, double _defaultValue, const QString &_label, const QString &_toolTip,
                                bool _hasValidator, double _validMin, double _validMax)
{
    if(!parameter_dictionary_.contains(_name)){
        Parameter *p = new Parameter(_defaultValue);
        if(_hasValidator){
            p->setHasValidator(_hasValidator);
            p->setValidMin((QVariant)_validMin);
            p->setValidMax((QVariant)_validMax);
        }
        addParameter(_name, p, _label, _toolTip);
    }
}

void ParameterSet::addParameter(const QString &_name, QStringList &_defaultValue, int _defaultIndex, const QString &_label, const QString &_toolTip)
{
    if(!parameter_dictionary_.contains(_name)){
        Parameter *p = new Parameter(_defaultValue, _defaultIndex);
        addParameter(_name, p, _label, _toolTip);
    }
}

QMap<QString, Parameter *> ParameterSet::getParameterDictionary() const
{
    return parameter_dictionary_;
}

QVector<Parameter *> ParameterSet::getAllParameters() const
{
    return all_parameters_;
}

void ParameterSet::removeAllParameter()
{
    all_parameters_.clear();
    parameter_dictionary_.clear();
}

void ParameterSet::addParameter(const QString &_name, Parameter *parameter, const QString &_label, const QString &_toolTip)
{
    parameter->setName(_name);
    parameter->setLabel(_label);
    parameter->setToolTip(_toolTip);
    all_parameters_.push_back(parameter);
    parameter_dictionary_.insert(_name, parameter);
}
