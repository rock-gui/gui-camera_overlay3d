#include <iostream>
#include "CameraOverlay3D.hpp"
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <osg/MatrixTransform>
#include <osg/TextureRectangle>
#include <osg/TexMat>
#include <osg/io_utils>
#include <frame_helper/FrameHelper.h>
#include <osgDB/ReadFile>
#include <QTimer>
#include <osgGA/TrackballManipulator>

using namespace vizkit3d;

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
    (*colors)[0].set(1.f, 1.f, 1.f, 1.f);
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
    state->setMode(GL_DEPTH_TEST, ::osg::StateAttribute::OFF);
    state->setTextureAttributeAndModes(0, background_image, osg::StateAttribute::ON);
    state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);

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


void CameraOverlay3D::setCameraIntrinsics(frame_helper::CameraCalibration const &calib)
{
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
    //Frame might be compressed, convert it.
    frame_helper::FrameHelper helper;
    base::samples::frame::Frame temp;
    base::samples::frame::frame_mode_t mode = p->data.frame_mode == base::samples::frame::MODE_GRAYSCALE ? base::samples::frame::MODE_GRAYSCALE : base::samples::frame::MODE_BGR;
    temp.init(p->data.getWidth(), p->data.getHeight(), p->data.getDataDepth(), mode);
    helper.convertColor(p->data, temp);

    //Convert image to osg::Image
    osg::ref_ptr<osg::Image> osg_image = osg::ref_ptr<osg::Image>(new osg::Image);
    switch(temp.frame_mode){
    case base::samples::frame::MODE_BGR:
        osg_image->setImage(temp.getWidth(), temp.getHeight(), 1,
                         GL_BGR, GL_BGR, GL_UNSIGNED_BYTE, (unsigned char*) temp.getImageConstPtr(), osg::Image::NO_DELETE, 1 );
        break;
    case base::samples::frame::MODE_GRAYSCALE:
        osg_image->setImage(temp.getWidth(), temp.getHeight(), 1,
                         GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_BYTE, (unsigned char*) temp.getImageConstPtr(), osg::Image::NO_DELETE, 1 );
        break;
    default:
        std::cerr << "Image format '"<<temp.frame_mode<<"' not supported" << std::endl;
    }
    updateImage(osg_image);
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(CameraOverlay3D)

