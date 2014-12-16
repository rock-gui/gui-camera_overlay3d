#include <iostream>
#include "CameraOverlay3D.hpp"
#include <vizkit3d/Vizkit3DWidget.hpp>

using namespace vizkit3d;

struct CameraOverlay3D::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    base::samples::frame::Frame data;
};


CameraOverlay3D::CameraOverlay3D()
    : p(new Data)
{
}

CameraOverlay3D::~CameraOverlay3D()
{
    delete p;
}

void CameraOverlay3D::resetCamera()
{
    Vizkit3DWidget * widget = dynamic_cast<Vizkit3DWidget *>(this->parent());
    widget->setCameraEye(0,0,0);
    widget->setCameraLookAt(0,0,1);
}


void CameraOverlay3D::setCameraFrame(std::string const &frame)
{
    Vizkit3DWidget * widget = dynamic_cast<Vizkit3DWidget *>(this->parent());
    widget->setVisualizationFrame(QString::fromStdString(frame));
}

void CameraOverlay3D::setCameraIntrinsics(frame_helper::CameraCalibration const &calib)
{
    Vizkit3DWidget * widget = dynamic_cast<Vizkit3DWidget *>(this->parent());
    osg::Camera* camera = widget->getView(0)->getCamera();
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

    float fx = calib.fx;
    float fy = calib.fy;
    float s = 0;
    float width = calib.width;
    float height = calib.height;
    float cx = calib.cx;
    float cy = calib.cy;
    float znear = 1.0f;
    float zfar = 10000.0f;

    osg::Matrixd P;
    //FIXME: Maybe transpose?
    P.set(          2*fx/width,                      0,                              0,  0,
                    -2*s/width,            2*fy/height,                              0,  0,
                    (width - 2*cx)/width, (height - 2*cy)/height, -(zfar + znear)/(zfar - znear), -1,
                    0,                      0,   -(2*zfar*znear)/(zfar-znear),  0);

    camera->setProjectionMatrix(P);
}

osg::ref_ptr<osg::Node> CameraOverlay3D::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void CameraOverlay3D::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void CameraOverlay3D::updateDataIntern(base::samples::frame::Frame const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(CameraOverlay3D)

