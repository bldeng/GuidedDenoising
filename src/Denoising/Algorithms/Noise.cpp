#include "Noise.h"

Noise::Noise(DataManager *_data_manager, ParameterSet *_paramter_set)
{
    if((_paramter_set == NULL) || (_data_manager == NULL))
        return;

    parameter_set_ = _paramter_set;
    data_manager_ = _data_manager;

    initParameters();
}

void Noise::addNoise()
{
    if(data_manager_->getMesh().n_vertices() == 0)
        return;

    // get parameters
    double noise_level, impulsive_level;
    if(!parameter_set_->getValue(QString("Noise level"), noise_level))
        return;
    if(!parameter_set_->getValue(QString("Impulsive level"), impulsive_level))
        return;

    int noise_type_index, noise_direction_index;
    if(!parameter_set_->getStringListIndex(QString("Noise type"), noise_type_index))
        return;
    if(!parameter_set_->getStringListIndex(QString("Noise direction"), noise_direction_index))
        return;

    data_manager_->MeshToOriginalMesh();

    NoiseType noise_type = (noise_type_index == 0)? kGaussian : kImpulsive;
    NoiseDirection noise_direction = (noise_direction_index == 0)? kNormal : kRandom;

    // compute average length of mesh
    double average_length = 0.0;
    TriMesh mesh = data_manager_->getMesh();
    for(TriMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); e_it++)
        average_length += mesh.calc_edge_length(*e_it);
    double edge_numbers = (double)mesh.n_edges();
    average_length /= edge_numbers;

    // add noise
    double standard_derivation = average_length * noise_level;
    int impulsive_vertex_number = (int)(mesh.n_vertices() * impulsive_level);

    mesh.request_face_normals();
    mesh.request_vertex_normals();
    mesh.update_normals();

    std::vector<double> GaussianNumbers;
    std::vector<TriMesh::Normal> RandomDirections;
    std::vector<std::pair<int, double> > VertexListAndGaussianNumbers;

    if(noise_type == kGaussian){
        randomGaussianNumbers(0, standard_derivation, (int)mesh.n_vertices(), GaussianNumbers);
        if(noise_direction == kNormal){
            for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++){

                TriMesh::Point p = mesh.point(*v_it) + mesh.normal(*v_it) * GaussianNumbers[v_it->idx()];
                mesh.set_point(*v_it, p);
            }
        }
        else if(noise_direction == kRandom){
            randomDirections((int)mesh.n_vertices(), RandomDirections);
            for(TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++){
                int index = v_it->idx();
                TriMesh::Point p = mesh.point(*v_it) + RandomDirections[index] * GaussianNumbers[index];
                mesh.set_point(*v_it, p);
            }
        }
    }
    else if(noise_type == kImpulsive){
        randomImpulsiveNumbers(0,(int)mesh.n_vertices() - 1, impulsive_vertex_number, 0, standard_derivation, VertexListAndGaussianNumbers);
        if(noise_direction == kNormal){
            for(int i = 0; i < (int)VertexListAndGaussianNumbers.size(); i++){
                int index = VertexListAndGaussianNumbers[i].first;
                TriMesh::VertexHandle vh = mesh.vertex_handle(index);
                TriMesh::Point p =  mesh.point(vh) + mesh.normal(vh) * VertexListAndGaussianNumbers[i].second;
                mesh.set_point(vh, p);
            }
        }
        else if(noise_direction == kRandom){
            randomDirections(impulsive_vertex_number, RandomDirections);
            for(int i = 0; i < (int)VertexListAndGaussianNumbers.size(); i++){
                int index = VertexListAndGaussianNumbers[i].first;
                TriMesh::VertexHandle vh = mesh.vertex_handle(index);
                TriMesh::Point p =  mesh.point(vh) + RandomDirections[i] * VertexListAndGaussianNumbers[i].second;
                mesh.set_point(vh, p);
            }
        }
    }

    data_manager_->setMesh(mesh);
    data_manager_->setNoisyMesh(mesh);
    data_manager_->setDenoisedMesh(mesh);
}

void Noise::initParameters()
{
    parameter_set_->removeAllParameter();
    parameter_set_->addParameter(QString("Noise level"), 0.5, QString("Noise level"), QString("The level of noise be added to the mesh."),
                                 true, 0.0, 1.0);
    parameter_set_->addParameter(QString("Impulsive level"), 0.5, QString("Impulsive level"), QString("The percentage of vertex be added with noise."),
                                 true, 0.0, 1.0);

    QStringList strList_NoiseType;
    strList_NoiseType.push_back(QString("Gaussian"));
    strList_NoiseType.push_back(QString("Impulsive"));

    parameter_set_->addParameter(QString("Noise type"), strList_NoiseType, 0, QString("Noise type"), QString("the type of noise"));

    QStringList strList_NoiseDirection;
    strList_NoiseDirection.push_back(QString("Normal"));
    strList_NoiseDirection.push_back(QString("Random"));

    parameter_set_->addParameter(QString("Noise direction"), strList_NoiseDirection, 0, QString("Noise direction"), QString("the direction of noise"));

    parameter_set_->setName(QString("Noise"));
    parameter_set_->setLabel(QString("Noise"));
    parameter_set_->setIntroduction(QString("Noise -- Parameters"));
}

double Noise::generateRandomGaussian(double mean, double StandardDerivation)
{
    static double v1, v2, s;
    static int phase = 0;
    double x;

    if(phase == 0)
    {
        do
        {
            v1 = -1 + 2 * (double)rand() / (double) RAND_MAX;
            v2 = -1 + 2 * (double)rand() / (double) RAND_MAX;
            s = v1 * v1 + v2 * v2;
        }while (s >= 1 || s == 0);

        x = v1 * sqrt(-2 * log(s) / s);
    }
    else
        x = v2 * sqrt(-2 * log(s) / s);

    phase = 1 - phase;

    return x * StandardDerivation + mean;
}

TriMesh::Normal Noise::generateRandomDirection()
{
    double x, y, z, length;
    do{
        x = -1 + 2 * (double)rand() / (double) RAND_MAX;
        y = -1 + 2 * (double)rand() / (double) RAND_MAX;
        length = x * x + y * y;
    }while(length>1);

    const double r = 2 * std::sqrt(1 - length);

    x *= r;
    y *= r;
    z = 1 - 2 * length;

    return TriMesh::Normal(x, y, z);
}

void Noise::randomGaussianNumbers(double mean, double StandardDerivation, int number, std::vector<double> &RandomNumbers)
{
    RandomNumbers.resize(number, 0.0);

    srand((unsigned int)time(NULL));
    for(int i = 0; i < number; i++){
        RandomNumbers[i] = generateRandomGaussian(mean, StandardDerivation);
    }
}

void Noise::randomImpulsiveNumbers(int min, int max, int number, double mean, double StandardDerivation,
                                   std::vector<std::pair<int, double> > &VertexListAndRandomNumbers)
{
    int range = max - min + 1;
    if(number > range) return;

    VertexListAndRandomNumbers.resize(number, std::make_pair(0, 0.0));

    std::vector<double> randomNumbers;
    randomGaussianNumbers(mean, StandardDerivation, number, randomNumbers);

    srand((unsigned int)time(NULL));
    std::vector<int> rangeVector(range);
    for(int i = 0; i < range; i++)
        rangeVector[i] = min + i;

    srand((unsigned int)time(NULL));
    std::vector<int> vertexIndexList(number);
    for(int i = 0; i < number; i++){
        int pos = (int)((double)rand() / RAND_MAX * range);
        vertexIndexList[i] = rangeVector[pos];
        range--;
        std::swap(rangeVector[pos], rangeVector[range]);
    }

    for(int i = 0; i < number; i++)
        VertexListAndRandomNumbers[i] = std::make_pair(vertexIndexList[i], randomNumbers[i]);
}

void Noise::randomDirections(int number, std::vector<TriMesh::Normal> &RandomDirections)
{
    RandomDirections.resize(number, TriMesh::Normal(0.0, 0.0, 0.0));

    srand((unsigned int)time(NULL));
    for(int i = 0; i < number; i++){
        RandomDirections[i] = generateRandomDirection();
    }
}
