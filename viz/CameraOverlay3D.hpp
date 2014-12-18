#ifndef camera_overlay3d_CameraOverlay3D_H
#define camera_overlay3d_CameraOverlay3D_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/samples/Frame.hpp>
#include <frame_helper/Calibration.h>
#include <osgQt/GraphicsWindowQt>
#include <base/samples/RigidBodyState.hpp>
#include "FollowNodeMatrixManipulator.h"
#include <osgGA/TrackballManipulator>

namespace vizkit3d
{
class CameraOverlay3D
        : public vizkit3d::Vizkit3DPlugin<base::samples::frame::Frame>
        , boost::noncopyable
{
    Q_OBJECT
public:
    CameraOverlay3D();
    ~CameraOverlay3D();

    Q_INVOKABLE void resetCamera();
    Q_INVOKABLE void setCameraFrame(std::string const &frame);
    Q_INVOKABLE void setCameraIntrinsics(frame_helper::CameraCalibration const &calib);
    Q_INVOKABLE void setCameraIntrinsicsVect(std::vector<double> const &calib);
    Q_INVOKABLE void updateImageFromFile(std::string const &file_path);
    Q_INVOKABLE void updateData(base::samples::frame::Frame const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::frame::Frame>::updateData(sample);}

public slots:


protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::frame::Frame const& plan);

private:
    struct Data;
    Data* p;
    osg::ref_ptr<osg::Node> frustum_;
    osg::ref_ptr<osg::Geode> image_plane_;
    osg::ref_ptr<osg::Group> root_;
    ::osg::Camera* camera;
    osgViewer::View* view_;
    FollowNodeMatrixManipulator* followCam;
    osgGA::TrackballManipulator* trackball_manipulator;
    osg::ref_ptr<osg::Node> makeFrustumFromCamera( osg::Camera* camera );
    void createImagePlane(osg::Camera* camera, float distance);
    void updateImage(osg::ref_ptr<osg::Image> img);
};
}
#endif
