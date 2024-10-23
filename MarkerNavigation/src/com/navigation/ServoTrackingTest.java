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
	
	private BufferedReader in;
	private PrintWriter out;
	private ServerSocket server;
	private Socket client;
	private String msg;

	@Override
	public void initialize(){
		pointer = getApplicationData().createFromTemplate("pointer"); // stvaranje alata iz tool templatea
		pointer.attachTo(robot.getFlange()); // spajanje alta na prihvatnicu robota
		marker = getApplicationData().getFrame("/fixed_marker"); // ucitavanje fiksnog markera
		try {
			// stvaranje TCP/IP sserver socketa i stvaranje ulaznog i izlaznog streama podataka
			
			server = new ServerSocket(30000);
			client = server.accept();
			
			
			in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		    out = new PrintWriter(client.getOutputStream(), true);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
		

	public void run() {
		
		getApplicationControl().setApplicationOverride(0.7); // postavljanje brzine kretanja robota u programu
		
		SmartServo motion = new SmartServo(robot.getCurrentJointPosition()); // objekt za kontrolu servo kretanja
		motion.setMinimumTrajectoryExecutionTime(20e-3);
		
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode(); // objekt za kontrolu Impedance moda
		impedanceControlMode.parametrize(CartDOF.TRANSL).setStiffness(800);
		impedanceControlMode.parametrize(CartDOF.TRANSL).setDamping(0.5);
		/*if (motion.validateForImpedanceMode(robot)){
			motion.setMode(impedanceControlMode); // postavljanje robota u impedance mode
		}*/
		
		
		IMotionContainer container = pointer.getFrame("pointer_TCP").moveAsync(motion);	// motion container za upravljanje kretanjama	
		ISmartServoRuntime runtime = motion.getRuntime(); // servo runtime objekt za opravljanje parametricma servo kretanja
		
		while (true){
			try {
				msg = in.readLine(); // citanje poruke sa mreže
			} catch (IOException e) {
				e.printStackTrace();
				break;
			}

		    if (msg == null || msg == "0") { // ako nema poruke preskace se trenutni loop
			      //getLogger().info("No tracking data");
			      //out.println("No data recived!");
			      continue;
			    }
		    String[] msgSplit = msg.split(",");
		    double[] data = new double[msgSplit.length-1];
		    if (msgSplit.length == 1) {// ako poruka nije vazeca preskace se trenutni loop
		    	//getLogger().info("No tracking data");
		    	//out.println("No data recived!");
		    	continue;
		    }
		      
		    for (int i = 1; i < msgSplit.length;i++) { // parsanje poruke, podatci su odvojeni zarezom: flag,x,y,z,A,B,C
		      data[i-1] = Double.parseDouble(msgSplit[i]);
		    }
		    //kopiranje koordinatnog sustava pomicnog markera i tocke unutar tog markera
		    Frame mobile_frame = marker.getChild("mobile_frame").copy();
		    Frame target = marker.getChild("target").copyWithRedundancy();
		    
		    Vector trans = Vector.of(data[0], data[1], data[2]);
		    Rotation rot = Rotation.ofRad(data[3], data[4], data[5]);
		    Transformation fwdTransform = Transformation.of(trans, rot);
		    
		    mobile_frame.transform(fwdTransform); // transformacija koordinatnog sustava pomicnog markera  prema podatcima s kamere
		    
		    target.setParent(mobile_frame);
		    target.transform(Transformation.ofTranslation(0, 0, -30)); // definiranje tocke 30mm "iza" pomicnog markera
		    
		    runtime.setDestination(target); // zadavanje odredista servo controli
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