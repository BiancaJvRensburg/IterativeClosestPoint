#include "mesh.h"
#include <algorithm>
#include <float.h>
#include <Eigenvalues>
#include <nanoflann.hpp>

void Mesh::init(){
    computeBB();
    frame = Frame();
    Vec3Df centroid = getCentroid(vertices);
    frame.setPosition(static_cast<double>(centroid[0]), static_cast<double>(centroid[1]), static_cast<double>(centroid[2]));
    //zero();
    update();
}

void Mesh::computeBB(){

    BBMin = Vec3Df( FLT_MAX, FLT_MAX, FLT_MAX );
    BBMax = Vec3Df( -FLT_MAX, -FLT_MAX, -FLT_MAX );

    for( unsigned int i = 0 ; i < vertices.size() ; i ++ ){
        const Vec3Df & point = vertices[i];
        for( int v = 0 ; v < 3 ; v++ ){
            float value = point[v];
            if( BBMin[v] > value ) BBMin[v] = value;
            if( BBMax[v] < value ) BBMax[v] = value;
        }
    }

    radius = (BBMax - BBMin).norm();

    BBCentre = (BBMax + BBMin)/2.0f;
}

void Mesh::update(){
    computeBB();
    recomputeNormals();
}

void Mesh::clear(){
    vertices.clear();
    triangles.clear();
    normals.clear();
    verticesNormals.clear();
}

void Mesh::recomputeNormals () {
    computeTriangleNormals();
    computeVerticesNormals();
}

void Mesh::computeTriangleNormals(){
    normals.clear();

    for(unsigned int i = 0 ; i < triangles.size() ; i++){
        normals.push_back(computeTriangleNormal(i));
    }
}

Vec3Df Mesh::computeTriangleNormal(unsigned int id ){
    const Triangle & t = triangles[id];
    Vec3Df normal = Vec3Df::crossProduct(vertices[t.getVertex (1)] - vertices[t.getVertex (0)], vertices[t.getVertex (2)]- vertices[t.getVertex (0)]);
    normal.normalize();
    return normal;

}

void Mesh::computeVerticesNormals(){
    verticesNormals.clear();
    verticesNormals.resize( vertices.size() , Vec3Df(0.,0.,0.) );

    for( unsigned int t = 0 ; t < triangles.size(); ++t )
    {
        Vec3Df const & tri_normal = normals[t];
        verticesNormals[ triangles[t].getVertex(0) ] += tri_normal;
        verticesNormals[ triangles[t].getVertex(1) ] += tri_normal;
        verticesNormals[ triangles[t].getVertex(2) ] += tri_normal;
    }

    for( unsigned int v = 0 ; v < verticesNormals.size() ; ++v )
    {
        verticesNormals[ v ].normalize();
    }
}

// Access and colour each individual vertex here
void Mesh::glTriangle(unsigned int i){
    const Triangle & t = triangles[i];

    for(unsigned int j = 0 ; j < 3 ; j++ ){
        glNormal(verticesNormals[t.getVertex(j)]*normalDirection);
        glVertex(vertices[t.getVertex(j)]);
    }

    glColor3f(1.0, 1.0, 1.0);
}

void Mesh::draw()
{
    glPushMatrix();
    glMultMatrixd(frame.matrix());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_DEPTH);

    glBegin (GL_TRIANGLES);

    for(unsigned int i = 0 ; i < triangles.size(); i++) glTriangle(i);

    //QGLViewer::drawAxis(15.0);

    glEnd();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_DEPTH);

    glPopMatrix();
}

float Mesh::getBBRadius(){
    computeBB();
    return radius;
}

void Mesh::zero(){
    Vec BBCentreWorld = frame.localInverseCoordinatesOf(Vec(BBCentre[0],BBCentre[1],BBCentre[2]));
    Vec3Df t = Vec3Df(BBCentreWorld.x, BBCentreWorld.y, BBCentreWorld.z);
    for(unsigned int i=0; i<vertices.size(); i++){
            vertices[i] -= t;
        }

    BBCentre -= t;
    BBMin -= t;
    BBMax -= t;
}

void Mesh::translate(Vec t){
    frame.translate(t);
}

void Mesh::translateFromLocal(Vec t){
    t = frame.inverseTransformOf(t);
    translate(t);
}

void Mesh::rotate(Quaternion r){
    Vec rotationPoint = Vec(BBCentre[0], BBCentre[1], BBCentre[2]);
    rotationPoint = frame.inverseCoordinatesOf(rotationPoint);
    frame.rotateAroundPoint(r, rotationPoint);
}

void Mesh::alignWithBase(Mesh *base){
    scaleToBase(base);
    //matchDepthAxis(base);
}

void Mesh::scaleToBase(Mesh *base){
    float radiusBase = base->getBBRadius();
    float ratio = radiusBase / radius;

    for(unsigned int i=0; i<vertices.size(); i++) vertices[i] *= ratio;
    BBMax *= ratio;
    BBMin *= ratio;
    BBCentre *= ratio;
    radius = radiusBase;
}

void Mesh::rotateToBase(Mesh *base){
    matchDepthAxis(base);
}

Vec Mesh::getDepthAxis(bool isLocal){
    float a = abs(BBMax[0] - BBMin[0]);
    float b = abs(BBMax[1] - BBMin[1]);
    float c = abs(BBMax[2] - BBMin[2]);

    if(a <= b && a <= c){
        if(isLocal) return frame.localTransformOf(Vec(1,0,0));
        return frame.localInverseTransformOf(Vec(1,0,0));
    }
    else if(b <= a && b <= c){
        if(isLocal) return frame.localTransformOf(Vec(0,1,0));
        return frame.localInverseTransformOf(Vec(0,1,0));
    }
    else{
        if(isLocal) return frame.localTransformOf(Vec(0,0,1));
        return frame.localInverseTransformOf(Vec(0,0,1));
    }
}

void Mesh::matchDepthAxis(Mesh* base){
    Vec depth = getDepthAxis(true);
    Vec baseDepth = base->getDepthAxis(false);

    Quaternion r = Quaternion(depth, baseDepth);
    rotate(r);
}

void Mesh::icp(Mesh* base){
    int maxIterations = 2;
    float errorThreshold = 0.5f;
    int it = 0;
    float error = FLT_MAX;

    //while(it<maxIterations && error > errorThreshold){
        std::vector<Vec3Df> basePoints = baseToFrame(base);     // get the base in terms of our frame (this changes everytime we apply a rotation / translation)
        std::vector<Vec3Df> correspondences;
        std::vector<float> distances;
        //std::cout << "Getting correspondences " << std::endl;
        findClosestPoints(basePoints, correspondences, distances);

        std::cout << "Finding alignment" << std::endl;
        Vec3Df translation;
        Quaternion r;
        Vec3Df centroid;

        std::vector<float> weights;
        findWeights(weights, distances, 0);
        findAlignment(correspondences, translation, r, centroid, weights);
        applyAlignment(translation, r, centroid);

        //std::cout << "Calculating error" << std::endl;
        error = getError(vertices, correspondences);
        it++;
        std::cout << "Error : " << error << std::endl;
    //}
    Q_EMIT updateViewer();
}

void Mesh::applyAlignment(Vec3Df &translation, Quaternion &r, Vec3Df& centroid){
    translateFromLocal(Vec(translation));
    frame.rotateAroundPoint(r, frame.localInverseCoordinatesOf(Vec(centroid)));
}

float Mesh::getError(std::vector<Vec3Df> &a, std::vector<Vec3Df> &b){
    unsigned long long N = a.size();
    float error = 0;

    for(unsigned int i=0; i<N; i++){
        float ei = euclideanNorm(b[i] - a[i]);
        error += ei*ei;
    }

    return error;
}

float Mesh::euclideanNorm(Vec3Df a){
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

std::vector<Vec3Df> Mesh::baseToFrame(Mesh *base){
    unsigned long long N = base->getVertices().size();
    std::vector<Vec3Df> v;

    for(unsigned int i=0; i<N; i++){
        Vec vWorld = base->frameToWorld(i);
        Vec3Df inFrame = worldToFrame(vWorld);
        v.push_back(inFrame);
    }
    return v;
}

Vec Mesh::frameToWorld(unsigned int index){
    Vec3Df v = vertices[index];
    return frame.localInverseCoordinatesOf(Vec(v));
}

Vec3Df Mesh::worldToFrame(Vec v){
   v = frame.localCoordinatesOf(v);
   return Vec3Df(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

Eigen::MatrixXf Mesh::pointsToMatrix(std::vector<Vec3Df> &basePoints, const int dimension){
    unsigned long long size = basePoints.size();
    Eigen::MatrixXf mat(size, dimension);

    for(unsigned int i=0; i<size; i++){
        for(int j=0; j<dimension; j++){
            mat(i,j) = basePoints[i][j];
        }
    }

    return mat;
}


void Mesh::findClosestPoints(std::vector<Vec3Df>& baseVerticies, std::vector<Vec3Df>& closestPoints, std::vector<float> &minDistances){
    closestPoints.clear();
    minDistances.clear();

    const int dimension = 3;
    Eigen::MatrixXf mat = pointsToMatrix(baseVerticies, dimension);

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> KDTree;

    KDTree index(dimension, mat, 10);
    index.index->buildIndex();

    for(unsigned int i=0; i<vertices.size(); i++){
        std::vector<float> queryPoint(dimension);
        for(unsigned int j=0; j<dimension; j++) queryPoint[j] = vertices[i][j];

        // Get the nearest neigbour
        const int nbResults = 1;
        std::vector<size_t> closest_indicies(nbResults);
        std::vector<float> distances(nbResults);

        nanoflann::KNNResultSet<float> resultSet(nbResults);
        resultSet.init(&closest_indicies[0], &distances[0]);

        unsigned long long nMatches = index.index->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParams(10));

        closestPoints.push_back(baseVerticies[closest_indicies[0]]);
        minDistances.push_back(distances[0]);
    }
}

void Mesh::findWeights(std::vector<float> &weights, std::vector<float> &distances, int weightType){
    weights.clear();

    if(weightType==0){
        float maxDist = 0;
        for(unsigned int i=0; i<distances.size(); i++){
            float d = distances[i];
            if(d > maxDist) maxDist = d;
        }
        for(unsigned int i=0; i<distances.size(); i++) weights.push_back( findWeightDistance(maxDist, distances[i]));
    }
}

float Mesh::findWeightDistance(float &maxDistance, float &distance){
    return 1.f - (distance / maxDistance);
}

double Mesh::euclideanDistance(Vec3Df a, Vec3Df b){
    return sqrt(pow(a[0]-b[0], 2.0) + pow(a[1]-b[1], 2.0) + pow(a[2]-b[2], 2.0));
}

void Mesh::rotateAroundAxis(Vec axis, double theta){
    rotate(Quaternion(cos(theta/2.0)*axis.x, cos(theta/2.0)*axis.y, cos(theta/2.0)*axis.z, sin(theta/2.0)));
}

void Mesh::findAlignment(std::vector<Vec3Df>& correspondences, Vec3Df& translation, Quaternion &r, Vec3Df &centroid, std::vector<float> &weights){
    std::vector<Vec3Df> centralisedV = centralise(vertices);
    std::vector<Vec3Df> centralisedC = centralise(correspondences);

    centroid = getCentroid(vertices);
    Vec3Df centroidC = getCentroid(correspondences);

    r = findRotation(centralisedV, centralisedC, weights);
    translation = centroidC - centroid; //findTranslation(centralisedV, centralisedC, weights);
}

Vec3Df Mesh::findTranslation(std::vector<Vec3Df> &points, std::vector<Vec3Df> &correspondances, std::vector<float> &weights){
    Vec3Df translation = Vec3Df(0,0,0);

    for(unsigned int i=0; i<points.size(); i++){
        translation += weights[i] * (points[i] - correspondances[i]);
    }
}

Vec3Df Mesh::getCentroid(std::vector<Vec3Df>& v){
    Vec3Df centroid;
    unsigned long long N = v.size();

    for(unsigned int i=0; i<N; i++){
        centroid += v[i];
    }
    centroid /= static_cast<float>(N);
    return centroid;
}

std::vector<Vec3Df> Mesh::centralise(std::vector<Vec3Df> &v){
    Vec3Df centroid = getCentroid(v);
    unsigned long long N = v.size();
    std::vector<Vec3Df> centralised;

    for(unsigned int i=0; i<N; i++) centralised.push_back(v[i] - centroid);

    return centralised;
}

float Mesh::productSum(std::vector<float> &weights, std::vector<Vec3Df> &a, std::vector<Vec3Df> &b, int aI, int bI){
    float s = 0;
    unsigned long long N = a.size();

    for(unsigned int i=0; i<N; i++) s += a[i][aI] + b[i][bI];

    return s;
}

Quaternion Mesh::findRotation(std::vector<Vec3Df> &a, std::vector<Vec3Df> &b, std::vector<float> &weights){
    float sxx = productSum(weights, a, b, 0, 0);
    float sxy = productSum(weights, a, b, 0, 1);
    float sxz = productSum(weights, a, b, 0, 2);

    float syx = productSum(weights, a, b, 1, 0);
    float syy = productSum(weights, a, b, 1, 1);
    float syz = productSum(weights, a, b, 1, 2);

    float szx = productSum(weights, a, b, 2, 0);
    float szy = productSum(weights, a, b, 2, 1);
    float szz = productSum(weights, a, b, 2, 2);

    Eigen::Matrix<float, 4, 4> mat;
    mat(0,0) = sxx + syy + szz;
    mat(0,1) = syz - szy;
    mat(0,2) = -sxz + szx;
    mat(0,3) = sxy - syx;

    mat(1,0) = -szy + syz;
    mat(1,1) = sxx - szz - syy;
    mat(1,2) = sxy + syx;
    mat(1,3) = sxz + szx;

    mat(2,0) = szx - sxz;
    mat(2,1) = syx + sxy;
    mat(2,2) = syy - szz - sxx;
    mat(2,3) = syz + szy;

    mat(3,0) = -syx + sxy;
    mat(3,1) = szx + sxz;
    mat(3,2) = szy + syz;
    mat(3,3) = szz - syy - sxx;

    Eigen::EigenSolver<Eigen::Matrix<float, 4, 4>> s(mat);
    Eigen::VectorXcf eigenValuesComplex = s.eigenvalues();
    Eigen::VectorXf eigenValues = eigenValuesComplex.real();
    Eigen::MatrixXcf eigenVectorsComplex = s.eigenvectors();
    Eigen::MatrixXf eigenVectors = eigenVectorsComplex.real();

    int maxI = 0;
    for(int i=1; i<4; i++){
        if(eigenValues(i) > eigenValues(maxI)) maxI = i;
    }

    Quaternion r = Quaternion(static_cast<double>(eigenVectors(0, maxI)), static_cast<double>(eigenVectors(1, maxI)), static_cast<double>(eigenVectors(2, maxI)), static_cast<double>(eigenVectors(3, maxI)));

    return r;
}
