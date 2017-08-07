#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>

/**** Copy pasted from the 3d basic shapes example *******/
#include <Qt3DRender/qcamera.h>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qcameralens.h>

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QCommandLinkButton>
#include <QtGui/QScreen>

#include <Qt3DInput/QInputAspect>

#include <Qt3DExtras/qtorusmesh.h>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DRender/qmesh.h>
#include <Qt3DRender/qtechnique.h>
#include <Qt3DRender/qmaterial.h>
#include <Qt3DRender/qeffect.h>
#include <Qt3DRender/qtexture.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/qsceneloader.h>
#include <Qt3DRender/qpointlight.h>

#include <Qt3DCore/qtransform.h>
#include <Qt3DCore/qaspectengine.h>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qfirstpersoncameracontroller.h>

/***********************************************************/

/* Includes for the cylinder */

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QPhongMaterial>



namespace Ui {
class Classifier;
}

class Classifier : public QWidget
{
    Q_OBJECT

public:
    explicit Classifier(QWidget *parent = 0);
    ~Classifier();

public slots:
//    void setRoll(float angle);
//    void setPitch(float angle);
//    void setYaw(float angle);
//    void setCylinderTranslation(QVector3D &t);
    void transform(float pitch, float yaw, float roll, QString msg);



private:
    Ui::Classifier *ui;
    Qt3DExtras::Qt3DWindow *view;
    QWidget* container;
    Qt3DCore::QEntity *m_cylinderEntity;
    Qt3DCore::QEntity *x;
    Qt3DCore::QEntity *y;
    Qt3DCore::QEntity *z;

    Qt3DCore::QTransform *cylinderTransform;
    Qt3DCore::QTransform *xtransform;
    Qt3DCore::QTransform *ytransform;
    Qt3DCore::QTransform *ztransform;

    QMatrix4x4 m_matrix;

};

#endif // CLASSIFIER_H
