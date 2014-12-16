require 'vizkit'

viz = Vizkit.default_loader.CameraOverlay3D
vis_gui = Vizkit.default_loader.RobotVisualization
vis_gui.modelFile = "/home/dmronga/Repos/RockDev/bundles/aila/data/aila.urdf"
vis_gui.jointsSize = 0.02

viz.frame = "LeftCamera"
viz.resetCamera()

Vizkit.exec
