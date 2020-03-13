#include "viewer.h"
#include "Triangle.h"
#include "Vec3D.h"
#include <QGLViewer/manipulatedFrame.h>

Viewer::Viewer(QWidget *parent, StandardCamera *cam) : QGLViewer(parent) {
    Camera *c = camera();       // switch the cameras
    setCamera(cam);
    delete c;
    prevX = 0;
    prevY = 0;
    prevZ = 0;
    isCurve = false;
}

void Viewer::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glMultMatrixd(manipulatedFrame()->matrix());

    if(isCurve) curve->draw();

    glColor4f(1.0, 0, 0, mesh.getAlpha());
    mesh.draw();

    glColor4f(1., 1., 1., baseMesh.getAlpha());
    baseMesh.draw();

    glPopMatrix();
}

void Viewer::init() {
  restoreStateFromFile();
  setMouseTracking(true);

  viewerFrame = new ManipulatedFrame();
  setManipulatedFrame(viewerFrame);
  setAxisIsDrawn(false);

  glEnable(GL_LIGHTING);
  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  //initCurve();
}

void Viewer::openOFF(QString filename, Mesh &m, bool isBase) {
    std::vector<Vec3Df> &vertices = m.getVertices();
    std::vector<Triangle> &triangles = m.getTriangles();

    FileIO::openOFF(filename.toStdString(), vertices, triangles);

    m.init(viewerFrame);
    connect(&m, &Mesh::updateViewer, this, &Viewer::toUpdate);

    // Set the camera
    if(isBase){
        Vec c = m.getBBCentre();
        Vec3Df center = Vec3Df(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
        float radius = m.getBBRadius();
        updateCamera(center, radius);
    }
    else m.alignWithBase(baseMesh);

    std::cout << "File loaded " << std::endl;

    update();
}

void Viewer::saveOFF(const QString & filename){
    FileIO::saveOFF(filename.toStdString(), mesh.getVertices(), mesh.getTriangles());
}

void Viewer::initCurve(bool isMand){
    control.clear();
    if(isMand){
        control.push_back(Vec(-5, 5.31, -4.34849));
        control.push_back(Vec(-4.41,1.9687,-2.7316));
        control.push_back(Vec(-3.84341,-0.8484,-1.6));
        control.push_back(Vec(-2.53101,-2.57117,1.67488));
        control.push_back(Vec(-0.125591 , -2.97056 , 2.95159));
        control.push_back(Vec(2.53101,-2.57117,1.67488));
        control.push_back(Vec(3.84341,-0.8484,-1.6));
        control.push_back(Vec(4.41,1.9687,-2.7316));
        control.push_back(Vec(5, 5.31, -4.34849));
    }
    else{
        control.push_back(Vec(-9.11428,-6.45205,-245.5));
        control.push_back(Vec(-3.83517, -3.67523,-152.996));
        control.push_back(Vec(-8.89661,-1.74754,-71.3201));
        control.push_back(Vec(0.829161,2.00455,57.442));
        control.push_back(Vec(5.04728,5.46025,148.302));
        control.push_back(Vec(2.65498,3.14425,239.397));
    }

    constructCurve();
}

void Viewer::constructCurve(){
    curve = new Curve(control.size(), control);
    unsigned int nbU = 100;
    curve->generateCatmull(nbU);
    isCurve = true;
}

void Viewer::toUpdate(){
    update();
}

void Viewer::updateCamera(const Vec3Df & center, float radius){
    camera()->setSceneCenter(Vec(static_cast<double>(center[0]), static_cast<double>(center[1]), static_cast<double>(center[2])));
    camera()->setSceneRadius(static_cast<double>(radius*1.05f));
    camera()->showEntireScene();
}

void Viewer::registration(){
    mesh.icp(baseMesh);
    update();
}

void Viewer::registrationSingleStep(){
    mesh.icpSingleIteration(baseMesh);
}

void Viewer::rotateX(int position){
    int pos = position - prevX;
    prevX = position;

    double theta = static_cast<double>(pos) * M_PI/ 180.0 + M_PI;
    mesh.rotateAroundAxis(Vec(1,0,0), theta);
    update();
}

void Viewer::rotateY(int position){
    int pos = position - prevY;
    prevY = position;

    double theta = static_cast<double>(pos) * M_PI/ 180.0 + M_PI;
    mesh.rotateAroundAxis(Vec(0,1,0), theta);
    update();
}

void Viewer::rotateZ(int position){
    int pos = position - prevZ;
    prevZ = position;

    double theta = static_cast<double>(pos) * M_PI/ 180.0 + M_PI;
    mesh.rotateAroundAxis(Vec(0,0,1), theta);
    update();
}

void Viewer::autoRotate(){
    mesh.rotateToBase(baseMesh);
}

void Viewer::setBaseAlpha(int alpha){
    float a = static_cast<float>(alpha) / 100.f;
    baseMesh.setAlpha(a);
    update();
}

void Viewer::setMeshAlpha(int alpha){
    float a = static_cast<float>(alpha) / 100.f;
    mesh.setAlpha(a);
    update();
}

void Viewer::writeJSON(QJsonObject &json){
    QJsonObject jMesh;
    mesh.writeJSON(jMesh);
    json["mesh"] = jMesh;

    QJsonArray cntrlArray;
    for(unsigned int i=0; i<control.size(); i++){   // this is just
        QJsonArray v;
        v.append(curve->getControlPoint(i)->getX());
        v.append(curve->getControlPoint(i)->getY());
        v.append(curve->getControlPoint(i)->getZ());
        cntrlArray.append(v);
    }
    json["control points"] = cntrlArray;
}

void Viewer::readJSON(const QJsonObject &json){
    if(json.contains("control points") && json["control points"].isArray()){
        control.clear();
        QJsonArray controlArray = json["control points"].toArray();
        for(int i=0; i<controlArray.size(); i++){
            QJsonArray singleControl = controlArray[i].toArray();
            control.push_back(Vec(singleControl[0].toDouble(), singleControl[1].toDouble(), singleControl[2].toDouble()));
        }
        constructCurve();
    }

    if(json.contains("mesh") && json["mesh"].isObject()){
        QJsonObject meshObject = json["mesh"].toObject();
        mesh.readJSON(meshObject);
        mesh.init(viewerFrame);
        connect(&mesh, &Mesh::updateViewer, this, &Viewer::toUpdate);
    }

    Vec c = mesh.getBBCentre();
    Vec3Df center = Vec3Df(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
    float radius = mesh.getBBRadius();
    updateCamera(center, radius);
}
