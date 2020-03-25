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

    void alignWithBase(Mesh &base);

    std::vector<Vec3Df> &getVertices(){return vertices;}
    const std::vector<Vec3Df> &getVertices()const {return vertices;}

    std::vector<Triangle> &getTriangles(){return triangles;}
    const std::vector<Triangle> &getTriangles()const {return triangles;}

    // Varifold
    std::vector<Vec3Df>& getBarycentres(){ calculateBarycentres(); return barycentres; }
    std::vector<Vec3Df>& getNormals(){ return normals; }
    std::vector<Vec3Df>& getUnitNormals(){ return unitNormals; }

    Quaternion& getRotation(){ return totalRotation; }
    Vec& getTranslation(){ return totalTranslation; }
    Quaternion& getLastRotation(){ return lastRotation; }
    Vec& getLastTranslation(){ return lastTranslation; }
    float& getLastScale(){ return lastScale; }
    Vec getLocalCoordinate(Vec &p){ return frame.localCoordinatesOf(p); }
    Vec& getZeroedTranslation(){ return zeroedTranslation; }

    void setReferenceFrame(const Frame *ref){frame.setReferenceFrame(ref);}
    void init(const Frame *ref);
    void draw();
    void clear();
    float& getAlpha(){ return transparencyAlpha; }
    void setAlpha(float alpha){ transparencyAlpha = alpha; }

    float getBBRadius(){ return radius; }
    Vec getBBCentre(){ return frame.localInverseCoordinatesOf(Vec(BBCentre)); }

    typedef std::priority_queue< std::pair< float , int > , std::deque< std::pair< float , int > > , std::greater< std::pair< float , int > > > FacesQueue;

    void invertNormal(){normalDirection *= -1;}

    // ICP
    Vec getDepthAxis(bool isLocal);
    void printEulerAngles(const Quaternion &q);

    void icp(Mesh &base);
    void icpSingleIteration(Mesh &base);
    void rotateAroundAxis(Vec axis, double alpha);
    void rotateToBase(Mesh& base);
    void zero();

    // Varifold
    double singleMeshInnerSquareProduct(Vec3Df &normalB, Vec3Df& unitNormal);
    double singleMeshInnerProduct(std::vector<Vec3Df>& barycentresB, std::vector<Vec3Df>& normalsB, std::vector<Vec3Df>& unitNormalsB, unsigned int indexA, unsigned int indexB);

Q_SIGNALS:
    void updateViewer();

protected:
    void computeBB();

    void update();
    void recomputeNormals();

    void icpStep(Mesh &base);

    void rotate(Quaternion r);
    void translate(Vec t);                  // t defined in the world
    void translateFromLocal(Vec t);         // t defined inside the frame
    void uniformScale(float &s);

    void scaleToBase(Mesh& base);
    void matchDepthAxis(Mesh& base);

    std::vector<Vec3Df> baseToFrame(Mesh& base);
    Vec frameToWorld(unsigned int index);       // convert the vertice from local to world coordinates
    Vec3Df frameToWorld(Vec3Df v);
    Vec3Df worldToFrame(Vec v);                 // convert the vertice from world to local coordinates
    Vec3Df frameToWorldVector(Vec3Df v);
    Vec3Df worldToFrameVector(Vec v);
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

    //Varifold
    double kernelGaussian(Vec3Df &x, Vec3Df &y, double sigma);
    double kernelInvariant(Vec3Df &x, Vec3Df &y, double sigma);
    double innerProduct(Vec3Df &x, Vec3Df &y);      // probably already exists
    double varifoldMeshInnerProduct(Mesh &m1, Mesh &m2);
    double varifoldDistance(Mesh &m1, Mesh &m2);
    double varifoldSingleDistance(unsigned int indexA, unsigned int indexB, std::vector<Vec3Df>& barycentresB, std::vector<Vec3Df>& normalsB, std::vector<Vec3Df>& unitNormalsB);
    void findClosestPointsVarifold(std::vector<Vec3Df>& baseVertices, std::vector<Vec3Df>& closestPoints, std::vector<Vec3Df>& barycentresB, std::vector<Vec3Df>& normalsB, std::vector<Vec3Df>& unitNormalsB);
    void baseToFrameVarifold(Mesh& base, std::vector<Vec3Df>& v, std::vector<Vec3Df>& barycentresB, std::vector<Vec3Df>& normalsB, std::vector<Vec3Df>& unitNormalsB);

    std::vector <Vec3Df> vertices;
    std::vector <Triangle> triangles;
    std::vector <Vec3Df> barycentres;
    std::vector<Vec3Df> normals;
    std::vector<Vec3Df> unitNormals;
    std::vector<Vec3Df> verticesNormals;

    std::vector<Vec3Df> cor;
    std::vector<Vec3Df> vc;
    Vec3Df centroidCor, centroidVc;
    Vec3Df centroidStable;

    Vec3Df BBMin;
    Vec3Df BBMax;
    Vec3Df BBCentre;
    float radius;
    double scaleFactor;

    float distError;
    int normalDirection;

    float transparencyAlpha;

    Quaternion totalRotation;
    Vec totalTranslation;
    Quaternion lastRotation;
    Vec lastTranslation;
    float lastScale;
    Vec zeroedTranslation;

    Frame frame;        // can do rotations from the frame?
};

#endif // MESH_H

