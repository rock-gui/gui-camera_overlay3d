require 'vizkit'
require 'transformer/runtime'

Orocos::CORBA.name_service.ip = "127.0.0.1"
Orocos.initialize
Orocos.load_typekit('robot_frames')
Orocos.load_typekit('aruco')

urdf_file = "#{ENV["AILA_BUNDLE_DIR"]}/data/aila.urdf"
limits = "#{ENV["AILA_BUNDLE_DIR"]}/config/joint_limits/joint_limits_bodywithhead.yml"
camera_calib = Types::FrameHelper::CameraCalibration.new
camera_calib.fx = 523.55539
camera_calib.fy = 524.20812
camera_calib.cx = 376.17248
camera_calib.cy = 284.91054
camera_calib.d0 = -0.36903
camera_calib.d1 = 0.20010
camera_calib.d2 = 0.00035
camera_calib.d3 = -0.00019
camera_calib.width = 782.0
camera_calib.height = 582.0


calib = Types::FrameHelper::CameraCalibration.new


#############################################################

image_files = Dir.entries('images').map do |f| "images/#{f}" if f.split('.')[-1]=='png' end.compact!.reverse!

robot_root = "Rover_base"
camera_segment = "LeftCamera"


#############################################################
view3d = Vizkit.vizkit3d_widget
view3d.setAxes(false)
view3d.setCameraManipulator("Trackball")
view3d.grid.enabled = false
view3d.show

overlay = Vizkit.default_loader.CameraOverlay3D
#overlay.frame = "LeftCamera"
overlay.setCameraIntrinsics(camera_calib)

roboviz = Vizkit.default_loader.RobotVisualization
roboviz.modelFile = urdf_file
roboviz.jointsSize = 0.02
roboviz.frame = "Rover_base"

ctrl_gui=Vizkit.default_loader.ControlUi
ctrl_gui.initFromYaml(limits)

##############################################################
#For testing frame writing, export written frame to ruby port
Orocos.load_typekit 'base'
require 'frame_helper_ruby'
export_task = Orocos::RubyTaskContext.new "camera_overlay"
frame_sample = Types::Base::Samples::Frame::Frame.new
export_task.create_output_port 'frame', "/base/samples/frame/Frame"
export_task.configure
export_task.start

export_timer=Qt::Timer.new
export_timer.connect(SIGNAL("timeout()")) do
    puts "writing?"
    overlay.writeImage 'the_file.png'
    FrameHelper.load_frame 'the_file.png', frame_sample
    export_task.frame.write frame_sample
end
export_timer.start(100)
##############################################################


Orocos.run do 
  begin
    #Change images every 5 seconds
    t=Qt::Timer.new
    t.start(500)
    idx=0
    t.connect(t, SIGNAL('timeout()')) do
       current_image = image_files[idx]
       overlay.updateImageFromFile(current_image)
       idx = (idx+1) % image_files.size
       puts "Set image #{current_image}"
    end

    ctrl_gui.enableSendCBs(true)

    ctrl_gui.connect(SIGNAL('sendSignal()')) do
        data = ctrl_gui.getJoints()

        roboviz.updateData(data)
        segs = [camera_segment]
        segs.each do |seg|
            t = roboviz.getTranslation(seg, robot_root)
            r = roboviz.getRotation(seg, robot_root)
            view3d.setTransformation(seg, robot_root, t, r)
        end
    end

    sleep(5)

    Vizkit.exec
  rescue Exception => ex
    puts ex
  end
end

