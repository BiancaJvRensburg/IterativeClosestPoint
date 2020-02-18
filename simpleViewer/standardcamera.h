#ifndef STANDARDCAMERA_H
#define STANDARDCAMERA_H

#include <QGLViewer/camera.h>

class StandardCamera : public qglviewer::Camera {
public:
  StandardCamera();

  void changeOrthoFrustumSize(int delta);
  virtual void getOrthoWidthHeight(GLdouble &halfWidth,
                                   GLdouble &halfHeight) const;

private:
  float orthoSize;
};

#endif // STANDARDCAMERA_H
