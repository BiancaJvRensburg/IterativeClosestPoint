#include "mesh.h"
#include <algorithm>
#include <float.h>

void Mesh::init(){
    computeBB();
    frame = Frame();
    frame.setPosition(static_cast<double>(BBCentre[0]), static_cast<double>(BBCentre[1]), static_cast<double>(BBCentre[2]));
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
    alignWithBase(base);

    // NOTE THE ICP ITSELF MUST BE DONE ACCORDING TO THE WORLD COORDINATES

    // determine correspondances
            // correspondances : closest euclidean point
}

// Brute force
void Mesh::findClosestPoints(Mesh *base, std::vector<int>& closestPoints){     // closestPoint[i] is the index of the closest point to index i
    closestPoints.clear();
    std::vector<Vec3Df> baseVerticies = base->getVertices();

    for(unsigned int i=0; i<vertices.size(); i++){
        double minDist = DBL_MAX;
        int minIndex = -1;
        for(unsigned int j=0; j<baseVerticies.size(); j++){
            double d = euclideanDistance(vertices[i], baseVerticies[j]);
            if(d<minDist){
                minDist = d;
                minIndex = static_cast<int>(j);
            }
        }
        closestPoints.push_back(minIndex);
    }
}

double Mesh::euclideanDistance(Vec3Df a, Vec3Df b){
    return sqrt(pow(a[0]-b[0], 2.0) + pow(a[1]-b[1], 2.0) + pow(a[2]-b[2], 2.0));
}

void Mesh::rotateAroundAxis(Vec axis, double theta){
    rotate(Quaternion(cos(theta/2.0)*axis.x, cos(theta/2.0)*axis.y, cos(theta/2.0)*axis.z, sin(theta/2.0)));
}
