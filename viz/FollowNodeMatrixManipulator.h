#ifndef FOLLOWNODEMATRIXMANIPULATOR_H
#define FOLLOWNODEMATRIXMANIPULATOR_H

#include <osg/NodeCallback>
#include <osgGA/CameraViewSwitchManipulator>

//Taken from https://www.movesinstitute.org/Sullivan/OSGTutorials/osgFollowMe.htm


struct UpdateAccumlatedMatrix : public osg::NodeCallback
{
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        matrix = osg::computeWorldToLocal(nv->getNodePath() );
        traverse(node,nv);
    }
    osg::Matrix matrix;
};

struct TransformAccumulator
{
public:
    TransformAccumulator();
    bool attachToGroup(osg::Group* g);
    osg::Matrix getMatrix();
protected:
    osg::ref_ptr<osg::Group> parent;
    osg::Node* node;
    UpdateAccumlatedMatrix* mpcb;
};



class FollowNodeMatrixManipulator : public osgGA::CameraManipulator
{
public:
    FollowNodeMatrixManipulator( TransformAccumulator* ta);
    bool handle (const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&aa);
    void updateTheMatrix();
    virtual void setByMatrix(const osg::Matrixd& mat);
    virtual void setByInverseMatrix(const osg::Matrixd&mat);
    virtual osg::Matrixd getInverseMatrix() const;
    virtual osg::Matrixd getMatrix() const;
protected:
    ~FollowNodeMatrixManipulator() {}
    TransformAccumulator* worldCoordinatesOfNode;
    osg::Matrixd theMatrix;
};
#endif
