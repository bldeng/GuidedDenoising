#ifndef PARAMETERSET_H
#define PARAMETERSET_H

#include <QString>
#include <QList>
#include <QVariant>
#include <QMap>
#include <QVector>

class Parameter
{
public:
    enum Type{kQVariant = -1, kBool = 0, kInt = 1, kDouble = 2, kQStringList = 3};

    Parameter(const QVariant _defaultValue) : value_(_defaultValue), default_value_(_defaultValue), index_(0), default_index_(0), has_validator_(false), type_(kQVariant) {}
    Parameter(const bool _defaultValue) : value_(QVariant(_defaultValue)), default_value_(QVariant(_defaultValue)), index_(0), default_index_(0), has_validator_(false), type_(kBool) {}
    Parameter(const int _defaultValue) : value_(QVariant(_defaultValue)), default_value_(QVariant(_defaultValue)), index_(0), default_index_(0), has_validator_(false), type_(kInt) {}
    Parameter(const double _defaultValue) : value_(QVariant(_defaultValue)), default_value_(QVariant(_defaultValue)), index_(0), default_index_(0), has_validator_(false), type_(kDouble) {}
    Parameter(const QStringList &_defaultValue, const int _defaultIndex) : value_(QVariant(_defaultValue)), default_value_(QVariant(_defaultValue)), index_(_defaultIndex), default_index_(_defaultIndex), has_validator_(false), type_(kQStringList) {}
    ~Parameter();

public:
    Type getType() const {return type_;}
    QString getName() const {return name_;}
    void setName(const QString &_name) {name_ = _name;}
    QString getLabel() const {return label_;}
    void setLabel(const QString &_label) {label_ = _label;}
    QString getToolTip() const {return tool_tip_;}
    void setToolTip(const QString &_toolTip) {tool_tip_ = _toolTip;}

    void setValue(const QVariant &_value) {value_ = _value;}
    void setValue(const bool _value) {value_ = (QVariant)_value;}
    void setValue(const int _value) {
        if(has_validator_){
            if(_value > valid_max_.toInt()){
                value_ = valid_max_;
                return;
            }
            else if(_value < valid_min_.toInt()){
                value_ = valid_min_;
                return;
            }
        }
        value_ = (QVariant)_value;
    }
    void setValue(const double _value) {
        if(has_validator_){
            if(_value > valid_max_.toDouble()){
                value_ = valid_max_;
                return;
            }
            else if(_value < valid_min_.toDouble()){
                value_ = valid_min_;
                return;
            }
        }
        value_ = (QVariant)_value;
    }
    void setValue(const QStringList &_value) {value_ = (QVariant)_value;}
    QVariant getValueQVariant() const {return value_;}
    bool getValueBool() const {return value_.toBool();}
    int getValueInt() const {return value_.toInt();}
    double getValueDouble() const {return value_.toDouble();}
    QStringList getValueQStringList() const {return value_.toStringList();}

    void setStringListIndex(const int _index) {index_ = _index;}
    int getStringListIndex() const {return index_;}

    void setDefaultValue(QVariant _default_value) {default_value_ = _default_value;}
    void setDefaultIndex(int _default_index) {default_index_ = _default_index;}
    QVariant getDefaultValue() const {return default_value_;}
    int getDefaultIndex() const {return default_index_;}

    void resetDefaultValue() {value_ = default_value_; index_ = default_index_;}

    void setHasValidator(const bool _hasValidator) {has_validator_ = _hasValidator;}
    bool getHasValidator() const {return has_validator_;}

    void setValidMax(QVariant _validMax) {valid_max_ = _validMax;}
    QVariant getValidMax() const {return valid_max_;}

    void setValidMin(QVariant _validMin) {valid_min_ = _validMin;}
    QVariant getValidMin() const {return valid_min_;}

private:
    QVariant value_;
    QVariant default_value_;

    int index_;
    int default_index_;

    QString name_;
    QString label_;
    QString tool_tip_;

    bool has_validator_;
    QVariant valid_min_;
    QVariant valid_max_;

    Type type_;
};

class ParameterSet
{
public:
    ParameterSet();
    ~ParameterSet();

public:
    QString getName() const {return name_;}
    void setName(const QString &_name) {name_ = _name;}
    QString getLabel() const {return label_;}
    void setLabel(const QString &_label) {label_ = _label;}
    QString getIntroduction() const {return introduction_;}
    void setIntroduction(const QString &_introduction) {introduction_ = _introduction;}

    bool getValue(const QString &_name, QVariant &_value) const;
    void setValue(const QString &_name, const QVariant &_value);
    bool getValue(const QString &_name, bool &_value) const;
    void setValue(const QString &_name, const bool &_value);
    bool getValue(const QString &_name, int &_value) const;
    void setValue(const QString &_name, const int &_value);
    bool getValue(const QString &_name, double &_value) const;
    void setValue(const QString &_name, const double &_value);
    bool getValue(const QString &_name, QStringList &_value) const;
    void setValue(const QString &_name, const QStringList &_value);

    bool getStringListIndex(const QString &_name, int &_index) const;
    void setStringListIndex(const QString &_name, const int &_index);

    void addParameter(const QString &_name, bool _defaultValue, const QString &_label = "", const QString &_toolTip = "");
    void addParameter(const QString &_name, int _defaultValue, const QString &_label = "", const QString &_toolTip = "",
                      bool _hasValidator = false, int _validMin = -1e10, int _validMax = 1e10);
    void addParameter(const QString &_name, double _defaultValue, const QString &_label = "", const QString &_toolTip = "",
                      bool _hasValidator = false, double _validMin = -1.0e10, double _validMax = 1.0e10);
    void addParameter(const QString &_name, QStringList &_defaultValue, int _defaultIndex, const QString &_label = "", const QString &_toolTip = "");

    QMap<QString, Parameter*> getParameterDictionary() const;
    QVector<Parameter *> getAllParameters() const;
    void removeAllParameter();

private:
    void addParameter(const QString &_name, Parameter *parameter, const QString &_label, const QString &_toolTip);
    QString name_;
    QString label_;
    QString introduction_;
    QVector<Parameter *> all_parameters_;
    QMap<QString, Parameter*> parameter_dictionary_;
};

#endif // PARAMETERSET_H
