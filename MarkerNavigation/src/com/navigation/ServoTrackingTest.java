package com.navigation;


import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.IApplicationOverrideControl;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.*;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.RedundancyParam;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import javax.inject.Inject;
import java.lang.*;

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
public class ServoTrackingTest extends RoboticsAPIApplication {
	@Inject
	private LBRMed robot;
	
	private Tool pointer;
	private ObjectFrame marker;
	private Frame curr_pos;
	
	private BufferedReader in;
	private PrintWriter out;
	private ServerSocket server;
	private Socket client;
	private String msg;
	
	

	@Override
	public void initialize(){
		pointer = getApplicationData().createFromTemplate("pointer"); // creation of the tool from the template
		pointer.attachTo(robot.getFlange()); // attaching the tool to the robot flange
		marker = getApplicationData().getFrame("/fixed_marker"); // loading the fixed marker frame
		

		
		
		
		try {
			// Setup of the server socket for the communication with the camera program
			
			server = new ServerSocket(30000);
			client = server.accept();
			
			
			in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		    out = new PrintWriter(client.getOutputStream(), true);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
		

	public void run() {
		
		// Moving the robot to the initial position
		pointer.getFrame("TCP").move(ptp(marker.getChild("P3")).setJointVelocityRel(0.2));
		
		getApplicationControl().setApplicationOverride(0.5); // setting the speed override for the movement
		
		SmartServo motion = new SmartServo(robot.getCurrentJointPosition()); // creating the servo motion object
		motion.setMinimumTrajectoryExecutionTime(20e-3);

		// Creating impedance control mode and applying it to the motion
		/*CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode(); 
		impedanceControlMode.parametrize(CartDOF.TRANSL).setStiffness(800);
		impedanceControlMode.parametrize(CartDOF.TRANSL).setDamping(0.5);
		if (motion.validateForImpedanceMode(robot)){
			motion.setMode(impedanceControlMode);
		}*/
		
		
		IMotionContainer container = pointer.getFrame("TCP").moveAsync(motion);	// motion container for tuning the asincronous motion	
		ISmartServoRuntime runtime = motion.getRuntime(); // servo runtime object for changing motion parameters
		
		// Motion is run in a loop until it is stopped from the teach pendant
		while (true){
			try {
				msg = in.readLine(); // reading the message from the network
			} catch (IOException e) {
				e.printStackTrace();
				break;
			}

		    if (msg == null || msg == "0") { // skip current loop if message is empty 
			      //getLogger().info("No tracking data");
			      //out.println("No data recived!");
			      continue;
			    }
		    
		    // Checking and parsing the message with shape: flag,x,y,z,A,B,C
		    String[] msgSplit = msg.split(",");
		    double[] data = new double[msgSplit.length-1];
		    if (msgSplit.length == 1) {
		    	//getLogger().info("No tracking data");
		    	//out.println("No data recived!");
		    	continue;
		    }
		      
		    for (int i = 1; i < msgSplit.length;i++) {
		      data[i-1] = Double.parseDouble(msgSplit[i]);
		    }
		    // Copying the frames from the App data and transforming them with the data received from the camera.
		    Frame mobile_frame = marker.getChild("mobile_frame").copy();
		    Frame target = marker.getChild("target").copyWithRedundancy();
		    //getLogger().info(msg);
		    Vector trans = Vector.of(data[0], data[1], data[2]);
		    Rotation rot = Rotation.ofRad(data[3], data[4], data[5]);
		    Transformation fwdTransform = Transformation.of(trans, rot);
		    
		    mobile_frame.transform(fwdTransform); 
		    
		    // Defining the target within the mobile frame with the offset to be behind the marker and an angle so that tool marker is also visible
		    target.setParent(mobile_frame);
		    target.transform(Transformation.ofTranslation(0, 0, -40));
		    target.transform(Transformation.ofDeg(0, 0, 0, 0, 0, 45));
		    
		    runtime.setDestination(target); // Sending new destination to the servo control
		    
		    // Sending robots current position to the paython script over the network
		    curr_pos = runtime.getCurrentCartesianPosition(pointer.getFrame("TCP"), mobile_frame);
		    out.println(curr_pos.toString());
			}   
	    
	}
	
	@Override
	public void dispose(){
		try {
			server.close();
		
			client.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
}