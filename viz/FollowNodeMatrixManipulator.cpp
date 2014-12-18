#include "FollowNodeMatrixManipulator.h"

TransformAccumulator::TransformAccumulator()
{
    parent = NULL;
    node = new osg::Node;
    mpcb = new UpdateAccumlatedMatrix();
    node->setUpdateCallback(mpcb);
}

osg::Matrix TransformAccumulator::getMatrix()
{
    return mpcb->matrix;
}

bool TransformAccumulator::attachToGroup(osg::Group* g)
// Don't call this method from within a callback (see this post.)
{
    bool success = false;
    if (parent != NULL)
    {
        int n = parent->getNumChildren();
        for (int i = 0; i < n; i++)
        {
            if (node == parent->getChild(i) )
            {
                parent->removeChild(i,1);
                success = true;
            }
        }
        if (! success)
        {
            return success;
        }
    }
    g->addChild(node);
    return true;
}


FollowNodeMatrixManipulator::FollowNodeMatrixManipulator( TransformAccumulator* ta)
{
    worldCoordinatesOfNode = ta; theMatrix = osg::Matrixd::identity();
}

void FollowNodeMatrixManipulator::updateTheMatrix()
{
    theMatrix = worldCoordinatesOfNode->getMatrix();
}

osg::Matrixd FollowNodeMatrixManipulator::getMatrix() const
{
    return theMatrix;
}

osg::Matrixd FollowNodeMatrixManipulator::getInverseMatrix() const
{
    // rotate the matrix from Y up to Z up.
    osg::Matrixd m;
    m = theMatrix;// * osg::Matrixd::rotate(M_PI, osg::Vec3(0,1,0) ) * osg::Matrixd::rotate(M_PI, osg::Vec3(0,0,1));
    return m;
}

void FollowNodeMatrixManipulator::setByMatrix(const osg::Matrixd& mat)
{
    theMatrix = mat;
}

void FollowNodeMatrixManipulator::setByInverseMatrix(const osg::Matrixd& mat)
{
    theMatrix = osg::Matrix::inverse(mat);
}

bool FollowNodeMatrixManipulator::handle
(const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&aa)
{
    switch(ea.getEventType())
    {
    case (osgGA::GUIEventAdapter::FRAME):
    {
        updateTheMatrix();
        return false;
    }
    }
    return false;
}
