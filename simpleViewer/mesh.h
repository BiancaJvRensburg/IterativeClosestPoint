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

    void setReferenceFrame(const Frame *ref){frame.setReferenceFrame(ref);}

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
    void zero();

Q_SIGNALS:
    void updateViewer();

protected:
    void computeBB();
    void alignWithBase(Mesh* base);

    void update();
    void recomputeNormals();

    void rotate(Quaternion r);
    void translate(Vec t);                  // t defined in the world
    void translateFromLocal(Vec t);         // t defined inside the frame

    void scaleToBase(Mesh* base);
    void matchDepthAxis(Mesh* base);

    std::vector<Vec3Df> baseToFrame(Mesh* base);
    Vec frameToWorld(unsigned int index);       // convert the vertice from local to world coordinates
    Vec3Df worldToFrame(Vec v);                 // convert the vertice from world to local coordinates

    void findClosestPoints(std::vector<Vec3Df>& basePoints, std::vector<Vec3Df>& closestPoints);
    double euclideanDistance(Vec3Df a, Vec3Df b);

    void findAlignment(std::vector<Vec3Df>& correspondences, Vec3Df& translation);
    Vec3Df getCentroid(std::vector<Vec3Df>& v);
    std::vector<Vec3Df> centralise(std::vector<Vec3Df>& v);
    float productSum(std::vector<Vec3Df>& a, std::vector<Vec3Df>& b, int aI, int bI);
    Quaternion findRotation(std::vector<Vec3Df>& a, std::vector<Vec3Df>& b);

    void applyAlignment(Vec3Df& translation);

    float getError(std::vector<Vec3Df>& a, std::vector<Vec3Df>& b);
    float euclideanNorm(Vec3Df a);

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

