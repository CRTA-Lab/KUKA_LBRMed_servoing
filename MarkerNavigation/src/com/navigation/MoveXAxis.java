package com.navigation;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.med.deviceModel.LBRMed;

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
public class MoveXAxis extends RoboticsAPIApplication {
	@Inject
	private LBRMed robot;
	private Tool fantom;
	private ObjectFrame frame;
	private Frame target;

	@Override
	public void initialize() {
		fantom = getApplicationData().createFromTemplate("phantom2"); // creation of the tool from the template
		fantom.attachTo(robot.getFlange()); // attaching the tool to the robot flange
		frame = getApplicationData().getFrame("/zero_frame");
		target = getApplicationData().getFrame("/zero_frame/sweep_point").copyWithRedundancy();
		target.setParent(fantom.getDefaultMotionFrame(), false);
		target.transform(Transformation.ofTranslation(-20, 0, 0));
		target.setParent(frame, true);
	}

	@Override
	public void run() {
		
		fantom.move(lin(target).setJointVelocityRel(0.5));
	}
}