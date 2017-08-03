#include "classifier.h"
#include "ui_classifier.h"

Classifier::Classifier(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Classifier)
{
    ui->setupUi(this);

    view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
    container = this->createWindowContainer(view);
    QSize screenSize = view->screen()->size();
    container->setMinimumSize(QSize(200, 100));
    container->setMaximumSize(screenSize);

    ui->verticalLayout->addWidget(container);

    Qt3DInput::QInputAspect *input = new Qt3DInput::QInputAspect;
    view->registerAspect(input);

    // Root entity
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();

    // Camera
    Qt3DRender::QCamera *cameraEntity = view->camera();

    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(0, 0, 20.0f));
    cameraEntity->setUpVector(QVector3D(0, 1, 0));
    cameraEntity->setViewCenter(QVector3D(0, 0, 0));

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(cameraEntity->position());
    lightEntity->addComponent(lightTransform);

    // Cylinder shape data
    Qt3DExtras::QCylinderMesh *cylinder = new Qt3DExtras::QCylinderMesh();
    cylinder->setRadius(1);
    cylinder->setLength(3);
    cylinder->setRings(100);
    cylinder->setSlices(20);

    // CylinderMesh Transform
    cylinderTransform = new Qt3DCore::QTransform();
    cylinderTransform->setScale(1.5f);
    cylinderTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 45.0f));
//    cylinderTransform->setTranslation(QVector3D(
//                                          (float)view->size().height()/2-10,
//                                          (float)view->size().width()/2+10,
//                                          0.0f ));
    cylinderTransform->setTranslation(QVector3D(
                                          -5.0f,
                                          +3.4f,
                                          1.0f ));


    Qt3DExtras::QPhongMaterial *cylinderMaterial = new Qt3DExtras::QPhongMaterial();
    cylinderMaterial->setDiffuse(QColor(Qt::GlobalColor::red ));

    // Cylinder
    m_cylinderEntity = new Qt3DCore::QEntity(rootEntity);
    m_cylinderEntity->addComponent(cylinder);
    m_cylinderEntity->addComponent(cylinderMaterial);
    m_cylinderEntity->addComponent(cylinderTransform);

    m_cylinderEntity->setEnabled(true);



    view->setRootEntity(rootEntity);


    show();


}

Classifier::~Classifier()
{
    delete ui;
}

void Classifier::setCylinderTranslation(QVector3D *t)
{
    cylinderTransform->setTranslation(*t);
    m_cylinderEntity->addComponent(cylinderTransform);

}

