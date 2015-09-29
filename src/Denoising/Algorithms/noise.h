#ifndef NOISE_H
#define NOISE_H

#include "../parameterset.h"
#include "../datamanager.h"
#include <vector>
#include <utility>
#include <algorithm>
#include <QList>

class Noise
{
public:
    explicit Noise(DataManager *_data_manager, ParameterSet *_paramter_set);
    ~Noise() {}

public:
    void addNoise();

private:
    enum NoiseType{kGaussian, kImpulsive};
    enum NoiseDirection{kNormal, kRandom};

    void initParameters();

    double generateRandomGaussian(double mean, double StandardDerivation);
    TriMesh::Normal generateRandomDirection();

    void randomGaussianNumbers(double mean, double StandardDerivation, int number, std::vector<double> &RandomNumbers);
    void randomImpulsiveNumbers(int min, int max, int number,
                                  double mean, double StandardDerivation,
                                  std::vector< std::pair<int, double> > &VertexListAndRandomNumbers);
    void randomDirections(int number, std::vector<TriMesh::Normal> &RandomDirections);

private:
    ParameterSet *parameter_set_;
    DataManager *data_manager_;
};

#endif // NOISE_H
