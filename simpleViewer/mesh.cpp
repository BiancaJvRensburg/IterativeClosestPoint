#include "mesh.h"
#include <algorithm>
#include <float.h>
#include <Eigenvalues>
#include <nanoflann.hpp>

void Mesh::init(const Frame *ref){
    computeBB();

    frame = Frame();
    frame.setPosition(Vec(0,0,0));
    setReferenceFrame(ref);

    distError = FLT_MAX;
    transparencyAlpha = 0.5;

    zero();

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

    radius = (BBMax - BBMin).norm() / 2.0f;

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
    unitNormals.clear();

    for(unsigned int i = 0 ; i < triangles.size() ; i++){
        computeTriangleNormal(i);
    }
}

void Mesh::computeTriangleNormal(unsigned int id ){
    const Triangle & t = triangles[id];
    Vec3Df normal = Vec3Df::crossProduct(vertices[t.getVertex (1)] - vertices[t.getVertex (0)], vertices[t.getVertex (2)]- vertices[t.getVertex (0)]);
    normals.push_back(normal);
    normal.normalize();
    unitNormals.push_back(normal);
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

void Mesh::calculateBarycentres(){
    barycentres.clear();

    for(unsigned int i=0; i<triangles.size(); i++){
        Vec3Df c = (vertices[triangles[i].getVertex(0)] + vertices[triangles[i].getVertex(1)] + vertices[triangles[i].getVertex(2)]) / 3.f;
        barycentres.push_back(c);
    }
}

void Mesh::glTriangle(unsigned int i){
    const Triangle & t = triangles[i];

    for(unsigned int j = 0 ; j < 3 ; j++ ){
        glNormal(verticesNormals[t.getVertex(j)]*normalDirection);
        glVertex(vertices[t.getVertex(j)]);
    }
}

void Mesh::draw()
{
    glPushMatrix();
    glMultMatrixd(frame.matrix());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_DEPTH);

    glBegin (GL_TRIANGLES);

    for(unsigned int i = 0 ; i < triangles.size(); i++) glTriangle(i);

    glColor3f(1.f, 1.f, 1.f);

    glEnd();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_DEPTH);

    glPopMatrix();
}

void Mesh::zero(){
    for(unsigned int i=0; i<vertices.size(); i++) vertices[i] -= BBCentre;
    computeBB();
}

void Mesh::translate(Vec t){
    frame.translate(t);
}

void Mesh::translateFromLocal(Vec t){
    t = frame.inverseTransformOf(t);
    translate(t);
}

void Mesh::rotate(Quaternion r){
    Vec rotationPoint = Vec(static_cast<double>(BBCentre[0]), static_cast<double>(BBCentre[1]), static_cast<double>(BBCentre[2]));
    rotationPoint = frame.inverseCoordinatesOf(rotationPoint);
    frame.rotateAroundPoint(r, rotationPoint);
}

void Mesh::alignWithBase(Mesh &base){
    scaleToBase(base);
    //matchDepthAxis(base);
}

void Mesh::scaleToBase(Mesh &base){
    float radiusBase = base.getBBRadius();
    float ratio = radiusBase / radius;

    uniformScale(ratio);
}

void Mesh::uniformScale(float &s){
    for(unsigned int i=0; i<vertices.size(); i++) vertices[i] *= s;
    BBMax *= s;
    BBMin *= s;
    BBCentre *= s;
    radius *= s;
}

void Mesh::rotateToBase(Mesh &base){
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

void Mesh::matchDepthAxis(Mesh &base){
    Vec depth = getDepthAxis(true);
    Vec baseDepth = base.getDepthAxis(false);

    Quaternion r = Quaternion(depth, baseDepth);
    rotate(r);
}

void Mesh::icp(Mesh& base){
    int maxIterations = 50;
    //float errorThreshold = 0.1f;
    int it = 0;
    float deltaError = 1;
    float prevError = 0;

    while(it<maxIterations && deltaError > 0){
        icpStep(base);
        it++;
        deltaError = abs(prevError - distError);
        prevError = distError;
    }

    //std::cout << "Varifold distance : " << varifoldDistance(this, base) << std::endl;

    distError = FLT_MAX;
}

void Mesh::icpSingleIteration(Mesh &base){
    icpStep(base);
    //std::cout << "Varifold distance : " << varifoldDistance(this, base) << std::endl;
    Q_EMIT updateViewer();
}

void Mesh::icpStep(Mesh& base){    // get the base in terms of our frame (this changes everytime we apply a rotation / translation)
    std::vector<Vec3Df> basePoints, baseNormals, baseUnitNormals, baseBarycentres;
    std::vector<Vec3Df> correspondences;

    /*baseToFrameVarifold(base, basePoints, baseBarycentres, baseNormals, baseUnitNormals);
    findClosestPointsVarifold(basePoints, correspondences, baseBarycentres, baseNormals, baseUnitNormals);*/

    basePoints = baseToFrame(base);
    findClosestPoints(basePoints, correspondences);

    Quaternion r;
    float s;
    findAlignment(correspondences, r, s);

    std::vector<Vec3Df> worldVertices = vertices;
    backToWorld(worldVertices);
    backToWorld(correspondences);
    applyAlignment(r, s, worldVertices, correspondences);

    distError = getError(worldVertices, correspondences);
    std::cout << "Error : " << distError << std::endl;
}

void Mesh::applyAlignment(Quaternion &r, float &s, std::vector<Vec3Df> &worldVertices, std::vector<Vec3Df> &worldCorrespondances){
    frame.rotate(r);
    //uniformScale(s);
    translate(Vec(findTranslation(worldVertices, worldCorrespondances)));
}

float Mesh::getError(std::vector<Vec3Df> &a, std::vector<Vec3Df> &b){
    unsigned long long N = a.size();
    float error = 0;

    for(unsigned int i=0; i<N; i++){
        float ei = euclideanNorm(b[i] - a[i]);
        error += ei*ei;
    }

    return error / static_cast<float>(N);
}

float Mesh::euclideanNorm(Vec3Df a){
    return sqrt(euclideanNormSquared(a));
}

float Mesh::euclideanNormSquared(Vec3Df a){
    return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
}

std::vector<Vec3Df> Mesh::baseToFrame(Mesh &base){
    unsigned long long N = base.getVertices().size();
    std::vector<Vec3Df> v;

    for(unsigned int i=0; i<N; i++){
        Vec vWorld = base.frameToWorld(i);
        Vec3Df inFrame = worldToFrame(vWorld);
        v.push_back(inFrame);
    }
    return v;
}

void Mesh::baseToFrameVarifold(Mesh &base, std::vector<Vec3Df> &v, std::vector<Vec3Df>& barycentresB,  std::vector<Vec3Df> &normalsB, std::vector<Vec3Df> &unitNormalsB){
    v.clear();
    barycentresB.clear();
    normalsB.clear();
    unitNormalsB.clear();

    v = base.getVertices();
    barycentresB = base.getBarycentres();
    normalsB = base.getNormals();
    unitNormalsB = base.getUnitNormals();

    unsigned long long N = v.size();

    for(unsigned int i=0; i<N; i++) v[i] = worldToFrame(Vec(base.frameToWorld(v[i])));

    N = barycentresB.size();

    for(unsigned int i=0; i<N; i++){
        barycentresB[i] = worldToFrame(Vec(base.frameToWorld(barycentresB[i])));
        normalsB[i] = worldToFrameVector(Vec(base.frameToWorldVector(normalsB[i])));
        unitNormalsB[i] = worldToFrameVector(Vec(base.frameToWorldVector(unitNormalsB[i])));
    }
}

Vec Mesh::frameToWorld(unsigned int index){
    Vec3Df v = vertices[index];
    return frame.localInverseCoordinatesOf(Vec(v));
}

Vec3Df Mesh::frameToWorld(Vec3Df v){
    Vec c = frame.localInverseCoordinatesOf(Vec(v));
    return Vec3Df(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
}

Vec3Df Mesh::worldToFrame(Vec v){
   v = frame.localCoordinatesOf(v);
   return Vec3Df(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

Vec3Df Mesh::frameToWorldVector(Vec3Df v){
    Vec c = frame.localInverseTransformOf(Vec(v));
    return Vec3Df(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
}

Vec3Df Mesh::worldToFrameVector(Vec v){
   v = frame.localTransformOf(v);
   return Vec3Df(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

void Mesh::backToWorld(std::vector<Vec3Df> &v){
    for(unsigned int i=0; i<v.size(); i++){
        v[i] = frameToWorld(v[i]);
    }
}

Eigen::MatrixXd Mesh::pointsToMatrix(std::vector<Vec3Df> &basePoints, const int dimension){
    unsigned long long size = basePoints.size();
    Eigen::MatrixXd mat(size, dimension);

    for(unsigned int i=0; i<size; i++){
        for(int j=0; j<dimension; j++){
            mat(i,j) = static_cast<double>(basePoints[i][j]);
        }
    }

    return mat;
}


void Mesh::findClosestPoints(std::vector<Vec3Df>& baseVerticies, std::vector<Vec3Df>& closestPoints){
    closestPoints.clear();

    const int dimension = 3;
    Eigen::MatrixXd mat = pointsToMatrix(baseVerticies, dimension);

    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> KDTree;

    KDTree index(dimension, mat, 10);
    index.index->buildIndex();

    for(unsigned int i=0; i<vertices.size(); i++){
        std::vector<double> queryPoint(dimension);
        for(unsigned int j=0; j<dimension; j++) queryPoint[j] = static_cast<double>(vertices[i][static_cast<int>(j)]);

        // Get the nearest neigbour
        const int nbResults = 1;
        std::vector<size_t> closest_indicies(nbResults);
        std::vector<double> distances(nbResults);

        nanoflann::KNNResultSet<double> resultSet(nbResults);
        resultSet.init(&closest_indicies[0], &distances[0]);

        index.index->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParams(10));

        closestPoints.push_back(baseVerticies[closest_indicies[0]]);
    }
}

double Mesh::euclideanDistance(Vec3Df a, Vec3Df b){
    return sqrt(pow(a[0]-b[0], 2.0) + pow(a[1]-b[1], 2.0) + pow(a[2]-b[2], 2.0));
}

void Mesh::rotateAroundAxis(Vec axis, double theta){
    rotate(Quaternion(cos(theta/2.0)*axis.x, cos(theta/2.0)*axis.y, cos(theta/2.0)*axis.z, sin(theta/2.0)));
}


// Note : the problem is BEFORE HERE, SVD and eigen decomposition give the same rotation
void Mesh::findAlignment(std::vector<Vec3Df>& correspondences, Quaternion &r, float &s){
    std::vector<Vec3Df> centralisedV = centralise(vertices);
    std::vector<Vec3Df> centralisedC = centralise(correspondences);

    r = findRotation(centralisedV, centralisedC);
    s = findUniformScale(centralisedV, centralisedC);
}

// To be called after rotation is applied
Vec3Df Mesh::findTranslation(std::vector<Vec3Df> &worldVertices, std::vector<Vec3Df> &correspondances){
    Vec3Df centroid = getCentroid(worldVertices);
    Vec3Df centroidC = getCentroid(correspondances);
    centroidStable = centroid;

    /*centroidVc = centroid;
    centroidCor = centroidC;

    std::cout << "Centroid v : " << centroidVc[0] << " , " << centroidVc[1] << " , " << centroidVc[2] << std::endl;
    std::cout << "Centroid c : " << centroidCor[0] << " , " << centroidCor[1] << " , " << centroidCor[2] << std::endl;*/

    return centroidC - centroid;
}

float Mesh::findUniformScale(std::vector<Vec3Df> &v, std::vector<Vec3Df> &c){
    unsigned long long N = v.size();
    float ys = 0;
    float ps = 0;

    for(unsigned int i=0; i<N; i++){
        ys += euclideanNormSquared(c[i]);
        ps += euclideanNormSquared(v[i]);
    }

    return sqrt(ys/ps);
}

Vec3Df Mesh::getCentroid(std::vector<Vec3Df>& v){
    Vec3Df centroid(0.,0.,0.);
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

float Mesh::productSum(std::vector<Vec3Df> &a, std::vector<Vec3Df> &b, int aI, int bI){
    float s = 0;
    unsigned long long N = a.size();

    for(unsigned int i=0; i<N; i++) s += a[i][aI] * b[i][bI];

    return s / static_cast<float>(N);
}

Quaternion Mesh::findRotation(std::vector<Vec3Df> &a, std::vector<Vec3Df> &b){
    float sxx = productSum(a, b, 0, 0);
    float sxy = productSum(a, b, 0, 1);
    float sxz = productSum(a, b, 0, 2);

    float syx = productSum(a, b, 1, 0);
    float syy = productSum(a, b, 1, 1);
    float syz = productSum(a, b, 1, 2);

    float szx = productSum(a, b, 2, 0);
    float szy = productSum(a, b, 2, 1);
    float szz = productSum(a, b, 2, 2);

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
    shiftQuaternion(r);
    return r;
}

void Mesh::shiftQuaternion(Quaternion &q){
    /* A quaternion is usually defined as q = (s, x, y, z)
     * In libqglviewer its defined as q = (x, y, z, s)
     * We have to shift any quaternion result we have in order for it to fit the libqglviewer definition
    */

    Quaternion x = q;
    for(int i=0; i<4; i++){
        q[i] = x[(i+1)%4];
    }
}

void Mesh::printEulerAngles(const Quaternion &q){
    Eigen::Matrix<double, 4, 4> mat;
    Eigen::Matrix<double, 4, 4> matBar;

    mat(0,0) = q[0];
    mat(0,1) = -q[1];
    mat(0,2) = -q[2];
    mat(0,3) = -q[3];

    mat(1,0) = q[1];
    mat(1,1) = q[0];
    mat(1,2) = -q[3];
    mat(1,3) = q[2];

    mat(2,0) = q[2];
    mat(2,1) = q[3];
    mat(2,2) = q[0];
    mat(2,3) = -q[1];

    mat(3,0) = q[3];
    mat(3,1) = -q[2];
    mat(3,2) = q[1];
    mat(3,3) = q[0];

    matBar(0,0) = q[0];
    matBar(0,1) = -q[1];
    matBar(0,2) = -q[2];
    matBar(0,3) = -q[3];

    matBar(1,0) = q[1];
    matBar(1,1) = q[0];
    matBar(1,2) = q[3];
    matBar(1,3) = -q[2];

    matBar(2,0) = q[2];
    matBar(2,1) = -q[3];
    matBar(2,2) = q[0];
    matBar(2,3) = q[1];

    matBar(3,0) = q[3];
    matBar(3,1) = q[2];
    matBar(3,2) = -q[1];
    matBar(3,3) = q[0];

    std::cout << "Euler angles : " << std::endl;
    Eigen::Matrix<double, 4, 4> r = matBar * mat;
    std::cout << r;

}

/************************************************VARIFOLDS*************************************************************/
double Mesh::kernelGaussian(Vec3Df &x, Vec3Df &y, double sigma){
    double d = euclideanDistance(x,y);
    return exp(-d*d/sigma*sigma);
}

double Mesh::kernelInvariant(Vec3Df &x, Vec3Df &y, double sigma){
    double ip = innerProduct(x,y);
    return exp(-2.0*ip*ip/sigma*sigma);
}

double Mesh::innerProduct(Vec3Df &a, Vec3Df &b){
    return static_cast<double>(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]);
}

double Mesh::varifoldMeshInnerProduct(Mesh &m1, Mesh &m2){
    std::vector<Vec3Df> cf1 = m1.getBarycentres();
    std::vector<Vec3Df> cf2 = m2.getBarycentres();
    std::vector<Vec3Df> nf1 = m1.getUnitNormals();
    std::vector<Vec3Df> nf2 = m2.getUnitNormals();
    std::vector<Vec3Df> vf1 = m1.getNormals();
    std::vector<Vec3Df> vf2 = m2.getNormals();
    unsigned long long nm1 = cf1.size();
    unsigned long long nm2 = cf2.size();

    double sum = 0;
    double sigmaP = 0.5;
    double sigmaS = 0.5;

    for(unsigned long long i=0; i<nm1; i++){
        for(unsigned long long j=0; j<nm2; j++){
            double kp = kernelGaussian(cf1[i], cf2[j], sigmaP);
            double ks = kernelInvariant(nf1[i], nf2[j], sigmaS);
            sum += kp * ks * static_cast<double>(euclideanNorm(vf1[i])) * static_cast<double>(euclideanNorm(vf2[i]));
        }
    }

    return sum;
}

double Mesh::varifoldDistance(Mesh &m1, Mesh &m2){
    double m1prod = varifoldMeshInnerProduct(m1, m1);
    double m2prod = varifoldMeshInnerProduct(m2, m2);
    double m1m2prod = varifoldMeshInnerProduct(m1, m2);

    return sqrt(m1prod - 2.0 * m1m2prod  + m2prod);
}

double Mesh::singleMeshInnerSquareProduct(Vec3Df &normalB, Vec3Df &unitNormal){
    double sigmaS = 0.5;
    double ks = kernelInvariant(unitNormal, unitNormal, sigmaS);
    double l2norm = static_cast<double>(euclideanNorm(normalB));
    return ks * l2norm * l2norm;
}

double Mesh::singleMeshInnerProduct(std::vector<Vec3Df> &barycentresB, std::vector<Vec3Df> &normalsB, std::vector<Vec3Df> &unitNormalsB, unsigned int indexA, unsigned int indexB){
    Vec3Df cf1 = barycentres[indexA];
    Vec3Df cf2 = barycentresB[indexB];
    Vec3Df nf1 = unitNormals[indexA];
    Vec3Df nf2 = unitNormalsB[indexB];
    Vec3Df vf1 = normals[indexA];
    Vec3Df vf2 = normalsB[indexB];

    double sigmaP = 0.5;
    double sigmaS = 0.5;

    double kp = kernelGaussian(cf1, cf2, sigmaP);
    double ks = kernelInvariant(nf1, nf2, sigmaS);
    return kp * ks *  static_cast<double>(euclideanNorm(vf1)) * static_cast<double>(euclideanNorm(vf2));
}

double Mesh::varifoldSingleDistance(unsigned int indexA, unsigned int indexB, std::vector<Vec3Df> &barycentresB, std::vector<Vec3Df> &normalsB, std::vector<Vec3Df> &unitNormalsB){
    double m1prod = singleMeshInnerSquareProduct(normals[indexA], unitNormals[indexA]);
    double m2prod = singleMeshInnerSquareProduct(normalsB[indexB], unitNormalsB[indexB]);
    double m1m2prod = singleMeshInnerProduct(barycentresB, normalsB, unitNormalsB, indexA, indexB);

    return m1prod - 2.0 * m1m2prod + m2prod;
}

void Mesh::findClosestPointsVarifold(std::vector<Vec3Df> &baseVertices, std::vector<Vec3Df> &closestPoints, std::vector<Vec3Df> &barycentresB, std::vector<Vec3Df> &normalsB, std::vector<Vec3Df> &unitNormalsB){
    closestPoints.clear();
    calculateBarycentres();

        for(unsigned int i=0; i<vertices.size(); i++){
            double minDist = DBL_MAX;
            int minIndex = -1;
            for(unsigned int j=0; j<baseVertices.size(); j++){
                double d = abs(varifoldSingleDistance(i, j, barycentresB, normalsB, unitNormalsB));
                if(d<minDist){
                    minDist = d;
                    minIndex = static_cast<int>(j);
                }
            }
            closestPoints.push_back(baseVertices[static_cast<unsigned int>(minIndex)]);
        }
}


// JSON
void Mesh::writeJSON(QJsonObject &json){
    std::vector<Vec3Df> worldVertices = vertices;
    backToWorld(worldVertices);
    QJsonArray ver;
    for(unsigned int i=0; i<worldVertices.size(); i++){
        QJsonArray v;
        for(int j=0; j<3; j++) v.append(static_cast<double>(worldVertices[i][j]));
        ver.append(v);
    }
    json["vertices"] = ver;

    QJsonArray tri;
    for(unsigned int i=0; i<triangles.size(); i++){
        QJsonArray v;
        for(unsigned int j=0; j<3; j++) v.append(triangles[i][j]);
        tri.append(v);
    }
    json["triangles"] = tri;
}

void Mesh::readJSON(const QJsonObject &json){
    if(json.contains("vertices") && json["vertices"].isArray()){
        vertices.clear();
        QJsonArray vArray = json["vertices"].toArray();
        for(int i=0; i<vArray.size(); i++){
            QJsonArray singleV = vArray[i].toArray();
            vertices.push_back(Vec3Df(singleV[0].toDouble(), singleV[1].toDouble(), singleV[2].toDouble()));
        }
    }

    if(json.contains("triangles") && json["triangles"].isArray()){
        triangles.clear();
        QJsonArray tArray = json["triangles"].toArray();
        for(int i=0; i<tArray.size(); i++){
            QJsonArray singleT = tArray[i].toArray();
            triangles.push_back(Triangle(singleT[0].toInt(), singleT[1].toInt(), singleT[2].toInt()));
        }
    }

    update();
}
