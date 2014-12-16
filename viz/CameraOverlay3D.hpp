#ifndef camera_overlay3d_CameraOverlay3D_H
#define camera_overlay3d_CameraOverlay3D_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/samples/Frame.hpp>
#include <frame_helper/Calibration.h>
#include <base/samples/RigidBodyState.hpp>

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
    osg::Node* frustum_;
};
}
#endif
