#ifndef MESH_H
#define MESH_H

#include "Vec3D.h"
#include "Triangle.h"
#include <queue>
#include <QGLViewer/qglviewer.h>
#include <Eigenvalues>

using namespace qglviewer;

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

    // Varifold
    std::vector<Vec3Df>& getBarycentres(){ calculateBarycentres(); return barycentres; }
    std::vector<Vec3Df>& getNormals(){ return normals; }
    std::vector<Vec3Df>& getUnitNormals(){ return unitNormals; }

    void setReferenceFrame(const Frame *ref){frame.setReferenceFrame(ref);}

    void init(int id);
    void draw();

    void clear();

    float getBBRadius(){ return radius; }
    Vec getBBCentre(){ return frame.localInverseCoordinatesOf(Vec(BBCentre)); }

    typedef std::priority_queue< std::pair< float , int > , std::deque< std::pair< float , int > > , std::greater< std::pair< float , int > > > FacesQueue;

    void invertNormal(){normalDirection *= -1;}

    // ICP
    Vec getDepthAxis(bool isLocal);

    void icp(Mesh *base);
    void icpSingleIteration(Mesh *base);
    void rotateAroundAxis(Vec axis, double alpha);
    void rotateToBase(Mesh* base);
    void zero();

    // Varifold
    double singleMeshInnerSquareProduct(unsigned int indexA);
    double singleMeshInnerProduct(Mesh *m1, Mesh *m2, unsigned int indexA, unsigned int indexB);

Q_SIGNALS:
    void updateViewer();

protected:
    void computeBB();
    void alignWithBase(Mesh* base);

    void update();
    void recomputeNormals();

    void icpStep(Mesh *base);

    void rotate(Quaternion r);
    void translate(Vec t);                  // t defined in the world
    void translateFromLocal(Vec t);         // t defined inside the frame
    void uniformScale(float &s);

    void scaleToBase(Mesh* base);
    void matchDepthAxis(Mesh* base);

    std::vector<Vec3Df> baseToFrame(Mesh* base);
    Vec frameToWorld(unsigned int index);       // convert the vertice from local to world coordinates
    Vec3Df frameToWorld(Vec3Df v);
    Vec3Df worldToFrame(Vec v);                 // convert the vertice from world to local coordinates
    void backToWorld(std::vector<Vec3Df> &v);   // convert the verticies from this frame back to the world coordinates

    void findClosestPoints(std::vector<Vec3Df>& basePoints, std::vector<Vec3Df>& closestPoints);
    double euclideanDistance(Vec3Df a, Vec3Df b);
    Eigen::MatrixXd pointsToMatrix(std::vector<Vec3Df>& basePoints, const int dimension);
    void shiftQuaternion(Quaternion &q);


    void findAlignment(std::vector<Vec3Df>& correspondences, Quaternion &r, float &s);
    Vec3Df getCentroid(std::vector<Vec3Df>& v);
    std::vector<Vec3Df> centralise(std::vector<Vec3Df>& v);
    float productSum(std::vector<Vec3Df>& a, std::vector<Vec3Df>& b, int aI, int bI);
    Quaternion findRotation(std::vector<Vec3Df>& a, std::vector<Vec3Df>& b);
    void printEulerAngles(const Quaternion &q);
    float findUniformScale(std::vector<Vec3Df>& v, std::vector<Vec3Df>& c);
    Vec3Df findTranslation(std::vector<Vec3Df> &worldVertices, std::vector<Vec3Df>& correspondences);

    void applyAlignment(Quaternion& r, float &s, std::vector<Vec3Df> &worldVertices, std::vector<Vec3Df> &correspondances);

    float getError(std::vector<Vec3Df>& a, std::vector<Vec3Df>& b);
    float euclideanNorm(Vec3Df a);
    float euclideanNormSquared(Vec3Df a);

    void computeTriangleNormals();
    void computeTriangleNormal(unsigned int t);
    void computeVerticesNormals();
    void calculateBarycentres();
    void glTriangle(unsigned int i);
    void getColour();

    //Varifold
    double kernelGaussian(Vec3Df &x, Vec3Df &y, double sigma);
    double kernelInvariant(Vec3Df &x, Vec3Df &y, double sigma);
    double innerProduct(Vec3Df &x, Vec3Df &y);      // probably already exists
    double varifoldMeshInnerProduct(Mesh *m1, Mesh *m2);
    double varifoldDistance(Mesh *m1, Mesh *m2);
    double varifoldSingleDistance(Mesh *m1, Mesh *m2, unsigned int indexA, unsigned int indexB);
    void findClosestPointsVarifold(Mesh *m2, std::vector<Vec3Df>& baseVertices, std::vector<Vec3Df>& closestPoints);

    std::vector <Vec3Df> vertices;
    std::vector <Triangle> triangles;
    std::vector <Vec3Df> barycentres;
    std::vector<Vec3Df> normals;
    std::vector<Vec3Df> unitNormals;
    std::vector<Vec3Df> verticesNormals;

    Vec3Df BBMin;
    Vec3Df BBMax;
    Vec3Df BBCentre;
    float radius;

    float distError;
    int id;
    float red, green, blue;

    int normalDirection;

    Frame frame;        // can do rotations from the frame?
};

#endif // MESH_H

