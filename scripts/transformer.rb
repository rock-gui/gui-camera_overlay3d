require 'vizkit'
require 'transformer/runtime'


urdf_file = "#{ENV["AILA_BUNDLE_DIR"]}/data/aila.urdf"
limits = "#{ENV["AILA_BUNDLE_DIR"]}/config/joint_limits/joint_limits_bodywithhead.yml"

Orocos.load_typekit('robot_frames')

#Setup chains we want to publish
chains=[]
c=Types::RobotFrames::Chain.new
c.root_link = "base_link"
c.name = "cam"
c.tip_link = "LeftCamera"
chains.push c.dup

c.name = "left_arm"
c.tip_link = "Wrist_l"
chains.push c.dup

c.name = "right_arm"
c.tip_link = "Wrist_r"
chains.push c.dup


ctrl_gui=Vizkit.default_loader.ControlUi
ctrl_gui.initFromYaml(limits)

#Set up transformer config
chains.each do |c|
    Orocos.transformer.manager.conf.dynamic_transform "fk.#{c.name}",c.root_link => c.tip_link
end
Orocos.transformer.update_configuration_state

Orocos.run Transformer.broadcaster_name, 'robot_frames::ChainPublisher', :as => "fk" do
    fk = Orocos.name_service.get 'fk'
    fk.chains = chains
    dk.urdf_file = urdf_file
    fk.configure
    fk.start
    port_writer = fk.input.writer

    ctrl_gui.enableSendCBs(true)

    ctrl_gui.connect(SIGNAL('sendSignal()')) do
        port_writer.write(ctrl_gui.getJoints())
    end

    bc=Orocos.name_service.get(Transformer.broadcaster_name)
    bc.start
    sleep(1)
    bc.setConfiguration Orocos.transformer.configuration_state
    STDIN.readline
end

