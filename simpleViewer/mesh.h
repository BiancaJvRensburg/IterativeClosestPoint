#ifndef MESH_H
#define MESH_H

#include "Vec3D.h"
#include "Triangle.h"
#include <queue>
#include <QGLViewer/qglviewer.h>

using namespace qglviewer;

enum Side {INTERIOR, EXTERIOR};

class Mesh : public QObject
{
    Q_OBJECT

public:

    Mesh():normalDirection(1.){}
    Mesh(std::vector<Vec3Df> &vertices, std::vector<Triangle> &triangles): vertices(vertices), triangles(triangles), normalDirection(1.){
        update();
    }
    ~Mesh(){}

    std::vector<Vec3Df> &getVertices(){return vertices;}
    const std::vector<Vec3Df> &getVertices()const {return vertices;}

    std::vector<Triangle> &getTriangles(){return triangles;}
    const std::vector<Triangle> &getTriangles()const {return triangles;}

    std::vector< std::vector<unsigned int>> &getVertexNeighbours(){return vertexNeighbours;}
    const std::vector< std::vector<unsigned int>> &getVertexNeighbours()const {return vertexNeighbours;}

    std::vector< std::vector<unsigned int>> &getVertexTriangles(){return vertexTriangles;}
    const std::vector< std::vector<unsigned int>> &getVertexTriangles()const {return vertexTriangles;}

    void init();
    void draw();

    void clear();

    float getBBRadius();

    typedef std::priority_queue< std::pair< float , int > , std::deque< std::pair< float , int > > , std::greater< std::pair< float , int > > > FacesQueue;

    void invertNormal(){normalDirection *= -1;}

    // ICP
    Vec getDepthAxis(bool isLocal);

    void icp(Mesh *base);
    void rotateAroundAxis(Vec axis, double alpha);
    void rotateToBase(Mesh* base);

protected:
    void computeBB();
    void zero();
    void alignWithBase(Mesh* base);

    void update();
    void recomputeNormals();

    void rotate(Quaternion r);
    void translate(Vec t);

    void scaleToBase(Mesh* base);
    void matchDepthAxis(Mesh* base);

    void findClosestPoints(Mesh* base, std::vector<int>& closestPoints);
    double euclideanDistance(Vec3Df a, Vec3Df b);

    void computeTriangleNormals();
    Vec3Df computeTriangleNormal(unsigned int t);
    void computeVerticesNormals();
    void glTriangle(unsigned int i);
    void glTriangleSmooth(unsigned int i);
    void glTriangleFibInMand(unsigned int i);

    std::vector <Vec3Df> vertices;      // starting verticies
    std::vector <Triangle> triangles;       // starting triangles
    std::vector<Vec3Df> normals;
    std::vector<Vec3Df> verticesNormals;
    std::vector< std::vector<unsigned int>> vertexNeighbours;       // each vertex's neighbours
    std::vector< std::vector<unsigned int>> vertexTriangles;        // the triangles each vertex belongs to

    Vec3Df BBMin;
    Vec3Df BBMax;
    Vec3Df BBCentre;
    float radius;

    int normalDirection;

    Frame frame;        // can do rotations from the frame?
};

#endif // MESH_H

