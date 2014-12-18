require 'vizkit'
require 'transformer/runtime'
require 'pry'

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

#Setup chains we want to publish
chains=[]
c=Types::RobotFrames::Chain.new
c.root_link = "Rover_base"
c.name = "cam"
c.tip_link = "LeftCamera"
chains.push c.dup

c.name = "left_arm"
c.tip_link = "Wrist_l"
chains.push c.dup

c.name = "right_arm"
c.tip_link = "Wrist_r"
chains.push c.dup

#Set up transformer config
chains.each do |c|
    Orocos.transformer.manager.conf.dynamic_transform "fk.#{c.name}",c.root_link => c.tip_link
end

#############################################################

overlay = Vizkit.default_loader.CameraOverlay3D
overlay.frame = "LeftCamera"
overlay.setCameraIntrinsics(camera_calib)

roboviz = Vizkit.default_loader.RobotVisualization
roboviz.modelFile = urdf_file
roboviz.jointsSize = 0.02
roboviz.frame = "Rover_base"

ctrl_gui=Vizkit.default_loader.ControlUi
ctrl_gui.initFromYaml(limits)

##############################################################

Orocos.run Transformer.broadcaster_name, 'robot_frames::ChainPublisher' => "fk" do
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

    #Make robot controllable via gui. Update FK and visualization
    fk = Orocos.name_service.get 'fk'
    fk.chains = chains
    fk.urdf_file = urdf_file
    fk.configure
    fk.start
    port_writer = fk.input.writer

    ctrl_gui.enableSendCBs(true)

    ctrl_gui.connect(SIGNAL('sendSignal()')) do
        port_writer.write(ctrl_gui.getJoints())
        roboviz.updateData(ctrl_gui.getJoints())
    end

    sleep(5)

    #Orocos.transformer.update_configuration_state
    bc=Orocos.name_service.get(Transformer.broadcaster_name)
    #bc.start
    #sleep(1)
    #bc.setConfiguration Orocos.transformer.configuration_state
    Vizkit.exec
  rescue Exception => ex
    puts ex
  end
end

