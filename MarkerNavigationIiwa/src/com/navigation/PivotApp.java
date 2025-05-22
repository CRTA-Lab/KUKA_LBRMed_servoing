package com.navigation;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class PivotApp extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	private Tool pointer;
	private ObjectFrame point;

	@Override
	public void initialize() {
		pointer = getApplicationData().createFromTemplate("pointer_w_marker1"); // creation of the tool from the template
		pointer.attachTo(robot.getFlange()); // attaching the tool to the robot flange
		point = getApplicationData().getFrame("/pivot_point");
		
	}

	@Override
	public void run() {
		Frame target_init = point.copyWithRedundancy();
		pointer.getFrame("TCP_angle").moveAsync(ptp(target_init).setJointVelocityRel(0.15).setBlendingOri(0.05).setJointJerkRel(0.3));
		
		while (true){
			for (int i = -1;i<2;i++){
				for(int j = -1;j<2;j++){
					Frame target = point.copyWithRedundancy();
					target.transform(Transformation.ofDeg(0, 0, 0, 20*i, 0, 20*j));
					pointer.getFrame("TCP_angle").moveAsync(lin(target).setJointVelocityRel(0.15).setBlendingOri(0.05).setJointJerkRel(0.3));
				}
			}
			
		}
	}
}