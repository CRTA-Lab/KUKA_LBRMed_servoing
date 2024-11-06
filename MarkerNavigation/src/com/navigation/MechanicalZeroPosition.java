package com.navigation;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class MechanicalZeroPosition extends RoboticsAPIApplication {
	private LBR lbr;
	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot will move to the mechanical zero position.";
	private Tool pointer;
	private ObjectFrame marker;
	private ObjectFrame P1;
	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		
		pointer = getApplicationData().createFromTemplate("pointer_w_marker"); // creation of the tool from the template
		pointer.attachTo(lbr.getFlange()); // attaching the tool to the robot flange
		marker = getApplicationData().getFrame("/fixed_marker"); // loading the fixed marker frame
		
		P1 = getApplicationData().getFrame("/P3");
	}

	public void run() {
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
        
    	
		getLogger().info("Move to the mechanical zero position");
		/*PTP ptpToMechanicalZeroPosition = ptp(0,0,0,0,0,0,0);
		ptpToMechanicalZeroPosition.setJointVelocityRel(0.25);
		lbr.move(ptpToMechanicalZeroPosition);*/
		pointer.getFrame("TCP").move(ptp(P1).setJointVelocityRel(0.2));
	}


}
