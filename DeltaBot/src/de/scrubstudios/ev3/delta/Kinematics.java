package de.scrubstudios.ev3.delta;

public class Kinematics {

	public boolean isValidPos = true;

	// ROBOT GEOMETRY =============================================================================

	// radius from base center to shoulder joint
	public static final float BASE = 66.4f;

	// radius from effector center to wrist joint
	public static final float EE = 30;

	// length of upper arm (length from shoulder joint to elbow joint)
	public static final int BICEP = 80;

	// length of lower arm (length from elbow joint to wrist joint)
	public static final int FOREARM = 200;

	// length of upper arm squared
	public static final int BICEP_SQ = BICEP * BICEP;

	// length of lower arm squared
	public static final int FOREARM_SQ = FOREARM * FOREARM;

	// number of motor revolutions required for one rotation of shoulder joint
	public static final int GEAR_RATIO = 1; // FIXME

	// TRIGONOMETRIC CONSTANTS ====================================================================

	private static final float PI = 3.141592f;
	private static final float SQRT3 = (float) Math.sqrt( 3 );
	private static final float SIN120 = SQRT3 / 2;
	private static final float COS120 = -0.5f;
	private static final float TAN60 = SQRT3;
	private static final float SIN30 = 0.5f;
	private static final float TAN30 = 1 / SQRT3;
	private static final float D2R = PI / 180; // deg to rad
	private static final float R2D = 180 / PI; // rad to deg

	// CONSTRUCTOR ================================================================================

	public Kinematics() {
	}

	// INVERSE KINEMATICS =========================================================================

	// helper function for calculating shoulder joint angle theta on the YZ-plane
	public int CalcAngYZ(float x0, float y0, float z0) {

		y0 += EE - BASE; //
		float A = (x0 * x0 + y0 * y0 + z0 * z0 + BICEP_SQ - FOREARM_SQ) / (2 * z0);
		float B = y0 / z0;
		float a = B * B + 1;
		float b = -2 * A * B;
		float c = A * A - BICEP_SQ;
		float D = b * b - 4 * a * c;
		if ( D < 0 ) {
			isValidPos = false;
			return 0;
		}
		float y = (float) ((-b + Math.sqrt( D )) / (2 * a));
		float z = A - y * B;
		return (int) Math.rint( (Math.atan2( z, y ) * R2D * GEAR_RATIO) );
	}

	/*
	 * calculates shoulder joint angles theta1,2,3 for the desired XYZ position. For theta1 the world coordinate system
	 * (WCS) XYZ is used. For theta2 and theta3 the WCS is rotated about the Z-axis by +120° and -120° respectively
	 */
	public int[] calcIK(float x0, float y0, float z0) {

		int[] angles = new int[3];
		angles[0] = CalcAngYZ( x0, y0, z0 );
		angles[1] = CalcAngYZ( x0 * COS120 + y0 * SIN120, y0 * COS120 - x0 * SIN120, z0 );
		angles[2] = CalcAngYZ( x0 * COS120 - y0 * SIN120, y0 * COS120 + x0 * SIN120, z0 );

		return angles;
	}

	// FORWARD KINEMATICS =========================================================================

	public int[] calcFK(int t1, int t2, int t3) {
		
		int[] pos = new int[3];
		
		t1 *= D2R;
		t2 *= D2R;
		t3 *= D2R;
		
		float y1 = (float) -(BASE-EE + BICEP * Math.cos( t1));
		float z1 = (float) (-BICEP * Math.sin( t1 ));

		float y2 = (float) ((BASE-EE + BICEP * Math.cos( t2 )) * SIN30);
		float x2 = y2 * TAN60;
		float z2 = (float) (-BICEP * Math.sin( t2 ));

		float y3 = (float) ((BASE-EE + BICEP * Math.cos( t3 )) * SIN30);
		float x3 = -y3 * TAN60;
		float z3 = (float) (-BICEP * Math.sin( t3 ));

		float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

		float w1 = y1 * y1 + z1 * z1;
		float w2 = x2 * x2 + y2 * y2 + z2 * z2;
		float w3 = x3 * x3 + y3 * y3 + z3 * z3;

		// x = (a1*z + b1);
		float a1 = ((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)) / dnm;
		float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / (2 * dnm);

		// y = (a2*z + b2);
		float a2 = -((z2 - z1) * x3 - (z3 - z1) * x2) / dnm;
		float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / (2 * dnm);

		// a*z^2 + b*z + c = 0
		float a = a1 * a1 + a2 * a2 + 1;
		float b = 2 * (a1 * b1 + a2 * (b2 - y1) - z1);
		float c = b1 * b1 + (b2 - y1) * (b2 - y1) + z1 * z1 - FOREARM_SQ;

		// discriminant
		float d = b * b - 4 * a * c;
		if ( d < 0 ) {
			// TODO
		}

		float z0 = (float) (-(-b + Math.sqrt( d )) / (2 * a));
		float x0 = (-a1 * z0 + b1);
		float y0 = (-a2 * z0 + b2);

		pos[0] = (int) Math.round( x0 );
		pos[1] = (int) Math.round( y0 );
		pos[2] = (int) Math.round( z0 );

		return pos;
	}
	
}
