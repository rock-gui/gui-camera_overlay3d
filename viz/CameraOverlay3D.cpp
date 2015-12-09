#include <iostream>
#include "CameraOverlay3D.hpp"
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <osg/MatrixTransform>
#include <osg/TextureRectangle>
#include <osg/TexMat>
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <QTimer>
#include <osgGA/TrackballManipulator>
#include <osg/Material>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <osgDB/WriteFile>
#include <osg/CameraNode>
#include <OpenThreads/Mutex>

using namespace vizkit3d;

class SnapImageDrawCallback : public osg::CameraNode::DrawCallback
{
public:

    SnapImageDrawCallback()
    {
        _snapImageOnNextFrame = false;
        _image = new osg::Image;
    }

    /**
     * @brief Set weather to snap or not snap next rendered image
     * @param flag: True if you want to snap next image
     */
    void setSnapping(bool flag) { _snapImageOnNextFrame = flag; }
    bool getSnapping() const { return _snapImageOnNextFrame; }
    /**
     * @brief Set whether to continue or nor continue snapping after successful snap
     * @param flag: Reset (stop) snappig after successful snap?
     */
    void setResetSnapping(bool flag) { _resetSnappingAfterFrame = flag; }
    bool getResetSnapping() const { return _resetSnappingAfterFrame; }

    virtual void operator () (const osg::CameraNode& camera) const
    {
        if (!_snapImageOnNextFrame) return;

        int x,y,width,height;
        x = camera.getViewport()->x();
        y = camera.getViewport()->y();
        width = camera.getViewport()->width();
        height = camera.getViewport()->height();

        _imageMutex.lock();
        _image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);
        _lastSnap = base::Time::now();
        _imageMutex.unlock();

        if(_resetSnappingAfterFrame)
            _snapImageOnNextFrame = false;
    }

    /**
     * @brief Retrieve previsouly captured image
     * * Allocation of the variable 'buffer' is performed within the function.
     * 'setSnapImageOnNextFrame' must have been called before
     *
     * @param buffer uninitialized buffer to store image in
     * @param wait_for_new true if you want to wait for a new image (instead of using a previously snapped one)
     */
    void getImage(osg::ref_ptr<osg::Image>& buffer, bool wait_for_new=true){
        if(wait_for_new){
            waitForNew();
        }

        _imageMutex.lock();
        buffer = osg::ref_ptr<osg::Image>(new osg::Image(*_image));
        _imageMutex.unlock();
    }

    void waitForNew(){
        setSnapping(true);
        base::Time now = base::Time::now();
        while(_lastSnap < now){
            usleep(100);
        }
    }

    /**
     * @brief Write current previously captured image to disk
     * @param filename: Where to write to. extension should be an common image extension (eg. filename.png)
     */
    bool write(const std::string& filename, bool wait_for_new=true){
        if(wait_for_new){
            waitForNew();
        }

        _imageMutex.lock();
        if (osgDB::writeImageFile(*_image, filename)) {
            std::cout  << "Saved image to `" << filename << "`" << std::endl;
        }
        _imageMutex.unlock();
        return true;
    }

protected:

    std::string _filename;
    mutable bool        _snapImageOnNextFrame;
    mutable bool        _resetSnappingAfterFrame;
    mutable OpenThreads::Mutex _imageMutex;
    osg::ref_ptr<osg::Image> _image;
    mutable base::Time _lastSnap;
};

// Given a Camera, create a wireframe representation of its
// view frustum. Create a default representation if camera==NULL.

void frustum_at_depth(osg::Camera* camera, const float& distance, float &left, float &right,
                      float &top, float &bottom){
    osg::Matrixd proj;
    proj = camera->getProjectionMatrix();
    left = distance * (proj(2,0)-1.0) / proj(0,0);
    right = distance * (1.0+proj(2,0)) / proj(0,0);
    top = distance * (1.0+proj(2,1)) / proj(1,1);
    bottom = distance * (proj(2,1)-1.0) / proj(1,1);
}

osg::ref_ptr<osg::Node> CameraOverlay3D::makeFrustumFromCamera( osg::Camera* camera )
{
    // Projection and ModelView matrices
    osg::Matrixd proj;
    osg::Matrixd mv;
    assert(camera);

    proj = camera->getProjectionMatrix();
    mv = camera->getViewMatrix();


    // Get near and far from the Projection matrix.
    const double near = proj(3,2) / (proj(2,2)-1.0);
    const double far = 5.0;//proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    float nLeft, nRight, nTop, nBottom;
    frustum_at_depth(camera, near, nLeft, nRight, nTop, nBottom);

    // Get the sides of the far plane.
    float fLeft, fRight, fTop, fBottom;
    frustum_at_depth(camera, far, fLeft, fRight, fTop, fBottom);

    // Our vertex array needs only 9 vertices: The origin, and the
    // eight corners of the near and far planes.
    osg::ref_ptr<osg::Vec3Array> v = osg::ref_ptr<osg::Vec3Array>(new osg::Vec3Array);
    v->resize( 9 );
    (*v)[0].set( 0., 0., 0. );
    (*v)[1].set( nLeft, nBottom, near );
    (*v)[2].set( nRight, nBottom, near );
    (*v)[3].set( nRight, nTop, near );
    (*v)[4].set( nLeft, nTop, near );
    (*v)[5].set( fLeft, fBottom, far );
    (*v)[6].set( fRight, fBottom, far );
    (*v)[7].set( fRight, fTop, far );
    (*v)[8].set( fLeft, fTop, far );

    osg::ref_ptr<osg::Geometry> geom = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
    geom->setUseDisplayList( false );
    geom->setVertexArray( v );

    osg::ref_ptr<osg::Vec4Array> c = osg::ref_ptr<osg::Vec4Array>(new osg::Vec4Array);
    c->push_back( osg::Vec4( 1., 1., 1., 1. ) );
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    GLushort idxLines[8] = {
        0, 5, 0, 6, 0, 7, 0, 8 };
    GLushort idxLoops0[4] = {
        1, 2, 3, 4 };
    GLushort idxLoops1[4] = {
        5, 6, 7, 8 };
    geom->addPrimitiveSet( osg::ref_ptr<osg::DrawElementsUShort>(new osg::DrawElementsUShort( osg::PrimitiveSet::LINES, 8, idxLines )) );
    geom->addPrimitiveSet( osg::ref_ptr<osg::DrawElementsUShort>(new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops0 )) );
    geom->addPrimitiveSet( osg::ref_ptr<osg::DrawElementsUShort>(new osg::DrawElementsUShort( osg::PrimitiveSet::LINE_LOOP, 4, idxLoops1 )) );

    osg::ref_ptr<osg::Geode> geode = osg::ref_ptr<osg::Geode>(new osg::Geode);
    geode->addDrawable( geom );

    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );


    // Create parent MatrixTransform to transform the view volume by
    // the inverse ModelView matrix.
    osg::ref_ptr<osg::MatrixTransform> mt = osg::ref_ptr<osg::MatrixTransform>(new osg::MatrixTransform);
    mt->setMatrix( osg::Matrixd::inverse( mv ) );
    mt->addChild( geode );

    return mt;
}

void CameraOverlay3D::createImagePlane(osg::Camera* camera, float distance)
{
    float l, r, t, b;
    //Place image plane in frustum at distance
    frustum_at_depth(camera, distance, l, r, t, b);

    // create the box
    osg::ref_ptr<osg::Geometry> geom = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
    osg::ref_ptr<osg::Vec3Array> v = osg::ref_ptr<osg::Vec3Array>(new osg::Vec3Array);
    v->push_back( osg::Vec3(l,t,distance));
    v->push_back( osg::Vec3(r,t,distance));
    v->push_back( osg::Vec3(r,b,distance));
    v->push_back( osg::Vec3(l,b,distance));
    geom->setVertexArray(v);

    // Draw a four-vertex quad from the stored data.
    osg::ref_ptr<osg::DrawArrays> arrays = osg::ref_ptr<osg::DrawArrays>(new ::osg::DrawArrays(::osg::PrimitiveSet::QUADS,0,v->size()));
    geom->addPrimitiveSet(arrays);

    osg::ref_ptr<osg::Vec2Array> texcoords = osg::ref_ptr<osg::Vec2Array>(new osg::Vec2Array(4));
    (*texcoords)[0].set(0.0f, 0.0f);
    (*texcoords)[1].set(1.0f, 0.0f);
    (*texcoords)[2].set(1.0f, 1.0f);
    (*texcoords)[3].set(0.0f, 1.0f);
    geom->setTexCoordArray(0, texcoords);

    osg::ref_ptr<osg::Vec3Array> normals = osg::ref_ptr<osg::Vec3Array>(new osg::Vec3Array(1));
    (*normals)[0].set(0.0f, 0.0f, -1.0f);
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    osg::ref_ptr<osg::Vec4Array> colors = osg::ref_ptr<osg::Vec4Array>(new osg::Vec4Array(1));
    (*colors)[0].set(1.f, 1.f, 1.f, alpha_);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    image_plane_->removeDrawables(0,1);
    image_plane_->addDrawable(geom);
    image_plane_->dirtyBound();
}

void CameraOverlay3D::updateImage(osg::ref_ptr<osg::Image> img)
{
    osg::ref_ptr<osg::TextureRectangle> background_image = osg::ref_ptr<osg::TextureRectangle>(new osg::TextureRectangle(img));

    osg::ref_ptr<osg::TexMat> texmat = new osg::TexMat;
    texmat->setScaleByTextureRectangleSize(true);

    // setup state
    osg::ref_ptr<osg::StateSet> state = image_plane_->getOrCreateStateSet();
    state->setMode( GL_LIGHTING, ::osg::StateAttribute::OFF );
    state->setTextureAttributeAndModes(0, background_image, osg::StateAttribute::ON);
    state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);

    osg::Material *material = (osg::Material *) image_plane_->getStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
    if(!material){
        material = new osg::Material();
    }


    osg::ref_ptr<osg::Geometry> geom = (osg::Geometry*) image_plane_->getDrawable(0);
    osg::ref_ptr<osg::Vec4Array> colors = osg::ref_ptr<osg::Vec4Array>(new osg::Vec4Array(1));
    (*colors)[0].set(1.f, 1.f, 1.f, alpha_);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    image_plane_->getStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
    image_plane_->getStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);


    this->setDirty();
}

struct CameraOverlay3D::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    base::samples::frame::Frame data;
};


CameraOverlay3D::CameraOverlay3D()
    : p(new Data)
{
    root_ = osg::ref_ptr<osg::Group>(new osg::Group);
    image_plane_ = new osg::Geode();
    frustum_ = new osg::Node();
    alpha_ = 0.5;
    perform_undistortion_=false;
}

CameraOverlay3D::~CameraOverlay3D()
{
    delete p;
}


void CameraOverlay3D::setCameraFrame(std::string const &frame)
{
    Vizkit3DWidget * parent = dynamic_cast<Vizkit3DWidget *>(this->parent());
    parent->setVisualizationFrame(QString::fromStdString(frame));
}

void CameraOverlay3D::setCameraIntrinsics(frame_helper::CameraCalibration const &calib, bool perform_undistortion)
{
    perform_undistortion_ = perform_undistortion;

    intrinsics_ = cv::Mat(3,3,CV_64F);
    intrinsics_.at<double>(0,0) = calib.fx;
    intrinsics_.at<double>(0,1) = 0.0;
    intrinsics_.at<double>(0,2) = calib.cx;
    intrinsics_.at<double>(1,0) = 0.0;
    intrinsics_.at<double>(1,1) = calib.fy;
    intrinsics_.at<double>(1,2) = calib.cy;
    intrinsics_.at<double>(2,0) = 0.0;
    intrinsics_.at<double>(2,1) = 0.0;
    intrinsics_.at<double>(2,2) = 1.0;

    dist_coef_ = cv::Mat(1,5,CV_64F);
    dist_coef_.at<double>(0,0) = calib.d0;
    dist_coef_.at<double>(0,1) = calib.d1;
    dist_coef_.at<double>(0,2) = calib.d2;
    dist_coef_.at<double>(0,3) = calib.d3;
    dist_coef_.at<double>(0,4) = 0.0;

    Vizkit3DWidget * parent = dynamic_cast<Vizkit3DWidget*>(this->parent());
    assert(parent);

    float scale = 1;

    float fx = calib.fx*scale;
    float fy = calib.fy*scale;
    float s = 0;
    float height = calib.height*scale;
    float width = calib.width*scale;
    float cx = calib.cx*scale;
    float cy = calib.cy*scale;
    float znear = 0.1f;
    float zfar = 100.0f;

    /* Create new window */
    ::osg::DisplaySettings* ds = ::osg::DisplaySettings::instance().get();
    ::osg::ref_ptr< ::osg::GraphicsContext::Traits> traits = new ::osg::GraphicsContext::Traits;
    traits->windowName = "CameraOverlay";
    traits->windowDecoration = true;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();
    osgQt::GraphicsWindowQt* gw = new osgQt::GraphicsWindowQt(traits.get());

    //Create new View
    view_ = new osgViewer::View;
    parent->addView(view_);

    //Hack to get access to scene. Vizkit3dWidget does not give access to parent->root
    std::vector<osgViewer::Scene*> scenes;
    parent->getScenes(scenes, true);
    view_->setSceneData(scenes[0]->getSceneData());

    //Setup camera for new view. This is our calibrated camera.
    camera = view_->getCamera();
    camera->setGraphicsContext( gw );
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

    camera->setClearColor(::osg::Vec4(0.4, 0.3, 0.4, 1.0) );
    camera->setViewport( new ::osg::Viewport(0, 0, traits->width, traits->height) );

    assert(root_);
    //Make the camera follow the root of this plug-in
    osg::PositionAttitudeTransform* camFrameConversion = new osg::PositionAttitudeTransform();
    //This transform aligns the camera to look into the correct direction.
    //We expect the camera to be z: front, x: right, y: down
    camFrameConversion->setAttitude(osg::Quat(M_PI, osg::Vec3(0,1,0), M_PI, osg::Vec3(0,0,1), 0, osg::Vec3(0,1,0)));
    root_->addChild(camFrameConversion);
    TransformAccumulator* camWorldCoords = new TransformAccumulator();
    camWorldCoords->attachToGroup(camFrameConversion);

    followCam =
            new FollowNodeMatrixManipulator(camWorldCoords);

    view_->setCameraManipulator(followCam);



    QWidget* widget = gw->getGLWidget();
    widget->show();
    QTimer* timer = new QTimer();
    connect( timer, SIGNAL(timeout()), widget, SLOT(update()) );
    timer->start(10);

    widget->setSizePolicy( QSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding ) );
    widget->setObjectName(QString("View Widget"));


    osg::Matrixd P(  2.0*fx/width,                 0,                         0,  0,
                     2.0*s/width,     2.0*fy/height,                         0,  0,
                     1-(2*cx/width), 1-(2.0*cy/height), (zfar+znear)/(znear-zfar), -1,
                     0,                 0, 2*zfar*znear/(znear-zfar),  0);


    camera->setProjectionMatrix(P);
    //camera->setViewport( new osg::Viewport(0, 0, width, height) );

    frustum_ = makeFrustumFromCamera(camera);
    createImagePlane(camera, 1);
}

osg::ref_ptr<osg::Node> CameraOverlay3D::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    root_->addChild(frustum_);
    root_->addChild(image_plane_);
    return root_;
}

void CameraOverlay3D::updateImageFromFile(std::string const &file_path)
{
    osg::ref_ptr<osg::Image> osg_image = osgDB::readImageFile(file_path);
    updateImage(osg_image);
}

void CameraOverlay3D::updateMainNode ( osg::Node* node )
{
    osg::Group* group = static_cast<osg::Group*>(node);
}

void CameraOverlay3D::updateDataIntern(base::samples::frame::Frame const& value)
{
    p->data = value;

    base::samples::frame::frame_mode_t mode = p->data.frame_mode == base::samples::frame::MODE_GRAYSCALE ? base::samples::frame::MODE_GRAYSCALE : base::samples::frame::MODE_BGR;

    //Convert frame to raw BGR (OpenCV)
    input_frame_.init(p->data.getWidth(), p->data.getHeight(), p->data.getDataDepth(), mode);
    frame_helper_.convertColor(p->data, input_frame_);

    //Convert to openCV mat
    input_image_cv_ = frame_helper_.convertToCvMat(input_frame_);

    //Undistort and flip around x-axis
    if(perform_undistortion_){
        cv::undistort(input_image_cv_, undistorted_image_cv_, intrinsics_, dist_coef_);
        cv::flip(undistorted_image_cv_, rotated_image_cv_, 0);
    }
    else{
        cv::flip(undistorted_image_cv_, rotated_image_cv_, 0);
    }


    //Convert image to osg::Image
    osg::ref_ptr<osg::Image> osg_image = osg::ref_ptr<osg::Image>(new osg::Image);
    switch(mode){
    case base::samples::frame::MODE_BGR:
        osg_image->setImage(rotated_image_cv_.cols, rotated_image_cv_.rows, 3,
                            GL_LINE_STRIP, GL_BGR, GL_UNSIGNED_BYTE, (unsigned char*) rotated_image_cv_.data, osg::Image::NO_DELETE, 1 );
        break;
    case base::samples::frame::MODE_GRAYSCALE:
        osg_image->setImage(rotated_image_cv_.cols, rotated_image_cv_.rows, 1,
                            GL_LINE_STRIP, GL_LUMINANCE, GL_UNSIGNED_BYTE, (unsigned char*) rotated_image_cv_.data, osg::Image::NO_DELETE, 1 );
        break;
    default:
        std::cerr << "Image format '"<<mode<<"' not supported" << std::endl;
    }
    updateImage(osg_image);
}

void CameraOverlay3D::writeImage(std::string const &file_path){
    osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = new
            SnapImageDrawCallback();
    camera->setPostDrawCallback( (snapImageDrawCallback.get()));
    if(snapImageDrawCallback.get())
    {
        std::cout << "make screenshot" << std::endl;
        snapImageDrawCallback->write(file_path, true);
    }
    else
    {
        std::cout << "Warning: no make screenshot" << std::endl;
    }
}

void CameraOverlay3D::getImage(osg::ref_ptr<osg::Image>& buffer){
    osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = new
            SnapImageDrawCallback();
    camera->setPostDrawCallback( (snapImageDrawCallback.get()));
    if(snapImageDrawCallback.get())
    {
        snapImageDrawCallback->getImage(buffer, true);
    }
    else
    {
        throw std::runtime_error("Could not register callback");
    }
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(CameraOverlay3D)

