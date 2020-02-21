#include "mesh.h"
#include <algorithm>
#include <float.h>
#include <Eigenvalues>

void Mesh::init(){
    computeBB();
    frame = Frame();
    frame.setPosition(static_cast<double>(BBCentre[0]), static_cast<double>(BBCentre[1]), static_cast<double>(BBCentre[2]));
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
    float errorThreshold = 50.f;
    int it = 0;
    float error = FLT_MAX;

    //while(it<maxIterations && error > errorThreshold){
        std::vector<Vec3Df> basePoints = baseToFrame(base);     // get the base in terms of our frame (this changes everytime we apply a rotation / translation)
        std::vector<Vec3Df> correspondences;
        std::cout << "Getting correspondences " << std::endl;
        findClosestPoints(basePoints, correspondences);

        std::cout << "Finding alignment" << std::endl;
        Vec3Df translation;
        findAlignment(correspondences, translation);
        applyAlignment(translation);

        std::cout << "Calculating error" << std::endl;
        error = getError(vertices, correspondences);
        it++;
   // }
    Q_EMIT updateViewer();
}

void Mesh::applyAlignment(Vec3Df &translation){
    translateFromLocal(Vec(translation));
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

// Brute force
void Mesh::findClosestPoints(std::vector<Vec3Df>& baseVerticies, std::vector<Vec3Df>& closestPoints){
    closestPoints.clear();

    for(unsigned int i=0; i<vertices.size(); i++){
        double minDist = DBL_MAX;
        int minIndex = -1;
        for(unsigned int j=0; j<baseVerticies.size()-5; j+=5){
            double d = euclideanDistance(vertices[i], baseVerticies[j]);
            if(d<minDist){
                minDist = d;
                minIndex = static_cast<int>(j);
            }
        }
        closestPoints.push_back(baseVerticies[static_cast<unsigned int>(minIndex)]);
    }
}

double Mesh::euclideanDistance(Vec3Df a, Vec3Df b){
    return sqrt(pow(a[0]-b[0], 2.0) + pow(a[1]-b[1], 2.0) + pow(a[2]-b[2], 2.0));
}

void Mesh::rotateAroundAxis(Vec axis, double theta){
    rotate(Quaternion(cos(theta/2.0)*axis.x, cos(theta/2.0)*axis.y, cos(theta/2.0)*axis.z, sin(theta/2.0)));
}

void Mesh::findAlignment(std::vector<Vec3Df>& correspondences, Vec3Df& translation){
    std::vector<Vec3Df> centralisedV = centralise(vertices);
    std::vector<Vec3Df> centralisedC = centralise(correspondences);

    Vec3Df centroidV = getCentroid(vertices);
    Vec3Df centroidC = getCentroid(correspondences);

    Quaternion r = findRotation(centralisedV, centralisedC);
    //frame.rotateAroundPoint(r, frame.localInverseCoordinatesOf(Vec(centroidV)));

    translation = centroidC - centroidV;
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

float Mesh::productSum(std::vector<Vec3Df> &a, std::vector<Vec3Df> &b, int aI, int bI){
    float s = 0;
    unsigned long long N = a.size();

    for(unsigned int i=0; i<N; i++) s += a[i][aI] + b[i][bI];

    return s;
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
    mat(0,3) = sxy - syz;

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

    std::cout << "eigenvalues:" << std::endl;
    std::cout << eigenValues << std::endl;
    std::cout << maxI << " : " << eigenValues(maxI) << std::endl;
    std::cout << "eigenvectors=" << std::endl;
    std::cout << eigenVectors << std::endl;
    std::cout << "Max : " << std::endl;
    for(int i=0; i<4; i++) std::cout << eigenVectors(i, maxI) << std::endl;

    Quaternion r = Quaternion(eigenVectors(0, maxI), eigenVectors(1, maxI), eigenVectors(2, maxI), eigenVectors(3, maxI));

    return r;
}
