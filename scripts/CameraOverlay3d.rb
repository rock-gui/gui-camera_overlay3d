require 'vizkit'
urdf_file = "#{ENV["AILA_BUNDLE_DIR"]}/data/aila.urdf"

image_files = Dir.entries('images').map do |f| f if f.split('.')[-1]=='png' end.compact!.reverse!

overlay = Vizkit.default_loader.CameraOverlay3D
vis_gui = Vizkit.default_loader.RobotVisualization
vis_gui.modelFile = urdf_file
vis_gui.jointsSize = 0.02

viz.frame = "LeftCamera"
viz.resetCamera()

t=Qt::Timer.new
t.start(5)

t.connect(t, SIGNAL('timeout()')) do 
   current_image = image_files.pop
   if not current_image
       t.stop
   end

   overlay.updateImageFromFile(current_image)
end



Vizkit.exec
