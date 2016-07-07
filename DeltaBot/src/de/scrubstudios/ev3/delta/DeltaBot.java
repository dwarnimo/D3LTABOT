package de.scrubstudios.ev3.delta;

import java.rmi.RemoteException;

import de.scrubstudios.ev3.SimpleTouch;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class DeltaBot {

	Kinematics kin;

	Brick ev3;
	
	RegulatedMotor[] motors = new RegulatedMotor[3];
	RegulatedMotor poti;

	EV3IRSensor ir;
	EV3TouchSensor touch;
	SampleProvider samp;
	SimpleTouch touch1;

	public DeltaBot() {

		kin = new Kinematics();

		ev3 = BrickFinder.getLocal();
		
		motors[0] = new EV3LargeRegulatedMotor( ev3.getPort( "A" ) );
		motors[1] = new EV3LargeRegulatedMotor( ev3.getPort( "B" ) );
		motors[2] = new EV3LargeRegulatedMotor( ev3.getPort( "C" ) );

		poti = new EV3MediumRegulatedMotor( ev3.getPort( "D" ) );
		ir = new EV3IRSensor( ev3.getPort( "S1" ) );
		touch = new EV3TouchSensor( ev3.getPort( "S2" ) );
		samp = touch.getTouchMode();
		touch1 = new SimpleTouch( samp );

		calibrate();
	}

	private void calibrate() {

		for ( RegulatedMotor m : motors ) {
			int theta;
			while ( !touch1.isPressed() ) {
				theta = poti.getTachoCount();
				LCD.drawString( "Theta: " + theta, 1, 1 );
				m.rotateTo( theta, true );
				Delay.msDelay( 50 );
				LCD.clear();
			}
			m.resetTachoCount();
			poti.resetTachoCount();
			LCD.clear();
			Sound.beep();
			while ( touch1.isPressed() ) {
				Delay.msDelay( 50 );
			}
		}
		Delay.msDelay( 500 );
		Sound.beepSequenceUp();
	}

	private void setMotorSpeed(int speed) {

		for ( RegulatedMotor m : motors ) {
			m.setSpeed( speed );
		}
	}

	private void setMotorAcc(int acc) {

		for ( RegulatedMotor m : motors ) {
			m.setAcceleration( acc );
		}
	}

	private void setMotorAngles(int t1, int t2, int t3) {

		motors[0].rotateTo( t1, true );
		motors[1].rotateTo( t2, true );
		motors[2].rotateTo( t3, true );
	}

	private void setMotorAngles(int[] angles) {

		for ( int i = 0; i < motors.length; i++ ) {
			motors[i].rotateTo( angles[i] * kin.GEAR_RATIO, true );
		}
	}

	private int[] getMotorAngles() {

		int[] angles = new int[3];
		for ( int i = 0; i < motors.length; i++ ) {
			angles[i] = motors[i].getTachoCount();
		}
		return angles;
	}

	private void motorsWaitComplete() {

		while ( motors[0].isMoving() || motors[1].isMoving() || motors[2].isMoving() ) {
			Delay.msDelay( 10 );
		}
		ev3.getAudio().systemSound( 0 );
	}

	private void closePorts() {

		for ( RegulatedMotor m : motors ) {
			m.close();
		}
		ir.close();
		touch.close();
	}

	private void moveHome() {

		setMotorAngles( 0, 0, 0 );
	}

	private void moveToPos(int xd, int yd, int zd, int minDist) {

		ev3.getLED().setPattern( 2 );
		int[] pos0 = new int[3];
		int[] posd = new int[3];
		int[] angCurr = new int[3];
		angCurr = getMotorAngles();

		pos0 = kin.calcFK( angCurr[0], angCurr[1], angCurr[2] );
		System.out.println( pos0[0] );
		System.out.println( pos0[1] );
		System.out.println( pos0[2] );
		posd[0] = xd;
		posd[1] = yd;
		posd[2] = zd;

		float length = (float) Math.sqrt( (posd[0] - pos0[0]) * (posd[0] - pos0[0])
				+ (posd[1] - pos0[1]) * (posd[1] - pos0[1]) + (posd[2] - pos0[2]) * (posd[2] - pos0[2]) );
		float incX = (posd[0] - pos0[0]) / (length / minDist);
		float incY = (posd[1] - pos0[1]) / (length / minDist);
		float incZ = (posd[2] - pos0[2]) / (length / minDist);

		int[][] thetas = new int[(int) Math.floor( (length / minDist) + 2 )][3];

		for ( int i = 0; i < ((length / minDist) + 1); i++ ) {
			int[] thetas_ = new int[3];
			thetas_ = (kin.calcIK( pos0[0] + i * incX, pos0[1] + i * incY, pos0[2] + i * incZ ));
			thetas[i][0] = thetas_[0] * kin.GEAR_RATIO;
			thetas[i][1] = thetas_[1] * kin.GEAR_RATIO;
			thetas[i][2] = thetas_[2] * kin.GEAR_RATIO;
		}

		for ( int i = 0; i < thetas.length; i++ ) {
			// System.out.println( "Waypoint " + i + ":" + " t1 = " + thetas[i][0] + " t2 = " + thetas[i][1]
			// + " t3 = " + thetas[i][2] );

			setMotorAngles( thetas[i][0], thetas[i][1], thetas[i][2] );
			// Delay.msDelay( 50 );
		}
		while ( motors[0].isMoving() || motors[1].isMoving() || motors[2].isMoving() ) {

		}
		ev3.getLED().setPattern( 1 );
		ev3.getAudio().systemSound( 1 );
		Delay.msDelay( 250 );
		ev3.getLED().setPattern( 0 );

	}

	public static void main(String[] args) {

		DeltaBot delta = new DeltaBot();

		int[] angles = new int[3];
		angles = delta.kin.calcIK( 0, 0, -200 );
		for ( int i = 0; i < angles.length; i++ ) {
			System.out.println( angles[i] );
		}

		int[] pos = new int[3];
		pos = delta.kin.calcFK( angles[0], angles[1], angles[2] );
		for ( int i = 0; i < pos.length; i++ ) {
			System.out.println( pos[i] );
		}
	}
}
