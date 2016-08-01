package de.scrubstudios.ev3;

public class Point3 {

	public float x, y, z;

	public Point3() {
		x = 0;
		y = 0;
		z = 0;
	}
	
	public Point3(Point3 newPos) {
		x = newPos.x;
		y = newPos.y;
		z = newPos.z;
	}

	public Point3(float newX, float newY, float newZ) {
		x = newX;
		y = newY;
		z = newZ;
	}

	public void setXYZ(Point3 newPos) {
		x = newPos.x;
		y = newPos.y;
		z = newPos.z;
	}

	public void setXYZ(float newX, float newY, float newZ) {
		x = newX;
		y = newY;
		z = newZ;
	}
	
	public void setX(float newX) {
		x = newX;
	}

	public void setY(float newY) {
		y = newY;
	}

	public void setZ(float newZ) {
		z = newZ;
	}

	public Point3 getXYZ() {
		return new Point3(x, y, z);
	}

	public float getX() {
		return x;
	}

	public float getY() {
		return y;
	}
	
	public float getZ() {
		return z;
	}

	/*
	 * returns the squared distance between the current and the desired position
	 */
	public float getDistSquared(Point3 posDes) {
		return (x - posDes.x) * (x - posDes.x) 
			 + (y - posDes.y) * (y - posDes.y) 
			 + (z - posDes.z) * (z - posDes.z);
	}

	/*
	 * returns the distance between the current and the specified position
	 */
	public float getDist(Point3 posDes) {
		return (float) Math.sqrt(getDistSquared(posDes));
	}

	/*
	 * returns the norm direction vector pointing from the current to the desired position
	 */
	public Point3 normDir(Point3 posDes) {
		float dist = getDist(posDes);
		Point3 normDir = new Point3();
		normDir.setX((posDes.x - x) / dist);
		normDir.setY((posDes.y - y) / dist);
		normDir.setZ((posDes.z - z) / dist);
		return normDir;
	}

	/*
	 * rotate the coordinates of the point about the X-axis by the specified amount of radians
	 * positive angles result in counter-clockwise rotation
	 */
	public Point3 rotX(float angRad) {
		float newY = (float) (y * Math.cos(angRad) - z * Math.sin(angRad));
		float newZ = (float) (z * Math.cos(angRad) + y * Math.sin(angRad));
		return new Point3(x, newY, newZ);
	}
	
	/*
	 * rotate the coordinates of the point about the Y-axis by the specified amount of radians
	 * positive angles result in counter-clockwise rotation
	 */
	public Point3 rotY(float angRad) {
		float newX = (float) (x * Math.cos(angRad) + z * Math.sin(angRad));
		float newZ = (float) (z * Math.cos(angRad) - x * Math.sin(angRad));
		return new Point3(newX, y, newZ);
	}
	
	/*
	 * rotate the coordinates of the point about the Z-axis by the specified amount of radians
	 * positive angles result in counter-clockwise rotation
	 */
	public Point3 rotZ(float angRad) {
		float newX = (float) (x * Math.cos(angRad) + y * Math.sin(angRad));
		float newY = (float) (y * Math.cos(angRad) - x * Math.sin(angRad));
		return new Point3(newX, newY, z);
	}

	/*
	 * calculate dot product
	 */
	public float dot(Point3 pos2) {
		return x * pos2.x + y * pos2.y + z * pos2.z;
	}

	/*
	 *  returns a string of the XYZ coordinates rounded to two decimal places
	 */
	public String toString() {
		return String.format("%8.2f %8.2f %8.2f", x, y, z);
	}
}
