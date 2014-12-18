#include <iostream>
#include "CameraOverlay3D.hpp"
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <osg/MatrixTransform>
#include <osg/TextureRectangle>
#include <osg/TexMat>
#include <frame_helper/FrameHelper.h>
#include <osgDB/ReadFile>

using namespace vizkit3d;

// Given a Camera, create a wireframe representation of its
// view frustum. Create a default representation if camera==NULL.
osg::ref_ptr<osg::Node> CameraOverlay3D::makeFrustumFromCamera( osg::Camera* camera )
{
    // Projection and ModelView matrices
    osg::Matrixd proj;
    osg::Matrixd mv;
    if (camera)
    {
        proj = camera->getProjectionMatrix();
        mv = camera->getViewMatrix();
    }
    else
    {
        // Create some kind of reasonable default Projection matrix.
        proj.makePerspective( 30., 1., 1., 10. );
        // leave mv as identity
    }

    // Get near and far from the Projection matrix.
    const double near = proj(3,2) / (proj(2,2)-1.0);
    const double far = proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    const double nLeft = near * (proj(2,0)-1.0) / proj(0,0);
    const double nRight = near * (1.0+proj(2,0)) / proj(0,0);
    const double nTop = near * (1.0+proj(2,1)) / proj(1,1);
    const double nBottom = near * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    const double fLeft = far * (proj(2,0)-1.0) / proj(0,0);
    const double fRight = far * (1.0+proj(2,0)) / proj(0,0);
    const double fTop = far * (1.0+proj(2,1)) / proj(1,1);
    const double fBottom = far * (proj(2,1)-1.0) / proj(1,1);

    createImagePlane(nLeft, nRight, nTop, nBottom, near+0.00001);

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

void CameraOverlay3D::createImagePlane(float l, float r, float t, float b, float z)
{
    // create the box
    osg::ref_ptr<osg::Geometry> geom = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
    osg::ref_ptr<osg::Vec3Array> v = osg::ref_ptr<osg::Vec3Array>(new osg::Vec3Array);
    v->push_back( osg::Vec3(l,t,z));
    v->push_back( osg::Vec3(r,t,z));
    v->push_back( osg::Vec3(r,b,z));
    v->push_back( osg::Vec3(l,b,z));
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

    image_plane_->addDrawable(geom);
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
    //image_plane_->dirtyBound();

    //FIXME: Needed?
    //state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);

    //FIXME: Needed sth like this?
    //osg::ref_ptr<osg::Vec4Array> colors = osg::ref_ptr<osg::Vec4Array>(new osg::Vec4Array(1));
    //(*colors)[0].set(1.f, 1.f, 1.f, 1.f);
    //image_plane_->setColorArray(colors);
    //image_plane_->setColorBinding(osg::Geometry::BIND_OVERALL);
    //image_plane_->dirtyBound();
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
    image_plane_ = new osg::Geode();
    frustum_ = new osg::Node();
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
    this->setDirty();

    osg::Camera* camera = widget->getView(0)->getCamera();
    osg::CullStack::CullingMode cullingMode = camera->getCullingMode();
    cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
    camera->setCullingMode( cullingMode );
}


void CameraOverlay3D::setCameraFrame(std::string const &frame)
{
    Vizkit3DWidget * widget = dynamic_cast<Vizkit3DWidget *>(this->parent());
    widget->setVisualizationFrame(QString::fromStdString(frame));
}

void CameraOverlay3D::setCameraIntrinsicsVect(std::vector<double> const &calib)
{
    frame_helper::CameraCalibration c;
    c.fx = calib[0];
    c.fy = calib[1];
    c.cx = calib[2];
    c.cy = calib[3];
    c.d0 = calib[4];
    c.d1 = calib[5];
    c.d2 = calib[6];
    c.d3 = calib[7];
    c.width = calib[8];
    c.height = calib[9];

    setCameraIntrinsics(c);
}

void CameraOverlay3D::setCameraIntrinsics(frame_helper::CameraCalibration const &calib)
{
    Vizkit3DWidget * widget = dynamic_cast<Vizkit3DWidget*>(this->parent());
    assert(widget);
    osg::Camera* camera = widget->getView(0)->getCamera();
    assert(camera);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

    float fx = calib.fx;
    float fy = calib.fy;
    float s = 0;
    float height = widget->getView(0)->getCamera()->getViewport()->height();
    float width = calib.width * (height/calib.height);
    float cx = calib.cx;
    float cy = calib.cy;
    float znear = 1.0f;
    float zfar = 10000.0f;

    /*widget->setMinimumWidth(width);
    widget->setMinimumHeight(height);
    widget->setMaximumWidth(width);
    widget->setMaximumHeight(height);
    camera->setViewport( new osg::Viewport(0, 0, width, height) );*/

    osg::Matrixd P;
    //FIXME: Maybe transpose?
    P.set(          2*fx/width,                      0,                              0,  0,
                    -2*s/width,            -2*fy/height,                              0,  0,
          (width - 2*cx)/width, (height - 2*cy)/height, (-zfar - znear)/(zfar - znear), -1,
                    0,                      0,   (-2*zfar*znear)/(zfar-znear),  0);

    camera->setProjectionMatrix(P);

    frustum_ = makeFrustumFromCamera(camera);
//    root_->addChild(frustum_);
//    this->setDirty();
}

osg::ref_ptr<osg::Node> CameraOverlay3D::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    root_ = osg::ref_ptr<osg::Group>(new osg::Group);
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

