#include "standardcamera.h"

#include <QWheelEvent>

using namespace qglviewer;

StandardCamera::StandardCamera() {
  orthoSize = 1.0;
}

void StandardCamera::changeOrthoFrustumSize(int delta) {
  if (delta > 0)
    orthoSize *= static_cast<float>(1.1);
  else
    orthoSize /= static_cast<float>(1.1);
}

void StandardCamera::getOrthoWidthHeight(GLdouble &halfWidth,
                                         GLdouble &halfHeight) const {
    Camera::getOrthoWidthHeight(halfWidth, halfHeight);
}
