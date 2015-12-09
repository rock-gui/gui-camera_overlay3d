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
#include <frame_helper/FrameHelper.h>

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

    Q_INVOKABLE void setCameraFrame(std::string const &frame);
    Q_INVOKABLE void setCameraIntrinsics(frame_helper::CameraCalibration const &calib, bool perform_undistortion=true);
    Q_INVOKABLE void updateImageFromFile(std::string const &file_path);
    Q_INVOKABLE void updateData(base::samples::frame::Frame const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::frame::Frame>::updateData(sample);}
    Q_INVOKABLE void getImage(osg::ref_ptr<osg::Image>& buffer);
    Q_INVOKABLE void writeImage(std::string const &file_path);
    inline osgViewer::View* view(){return view_;}

public slots:


protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(base::samples::frame::Frame const& plan);

private:
    struct Data;
    Data* p;

    base::samples::frame::Frame input_frame_;
    cv::Mat undistorted_image_cv_, rotated_image_cv_, input_image_cv_;
    cv::Mat intrinsics_, dist_coef_;
    frame_helper::FrameHelper frame_helper_;
    osg::ref_ptr<osg::Group> frustum_;
    osg::ref_ptr<osg::Geode> image_plane_;
    osg::ref_ptr<osg::Group> root_;
    ::osg::Camera* camera;
    osgViewer::View* view_;
    FollowNodeMatrixManipulator* followCam;
    osg::ref_ptr<osg::Node> makeFrustumFromCamera( osg::Camera* camera );
    void createImagePlane(osg::Camera* camera, float distance);
    void updateImage(osg::ref_ptr<osg::Image> img);
    float alpha_;
    bool perform_undistortion_;
};
}
#endif
