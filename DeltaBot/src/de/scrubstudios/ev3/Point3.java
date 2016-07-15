package de.scrubstudios.ev3;

public class Point3 {

	public float x, y, z;

	public Point3() {
		x = 0;
		y = 0;
		z = 0;
	}

	public Point3(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Point3(Point3 pos) {
		x = pos.x;
		y = pos.y;
		z = pos.z;
	}

	public void setXYZ(float newX, float newY, float newZ) {
		x = newX;
		y = newY;
		z = newZ;
	}

	public void setXYZ(Point3 newPos) {
		x = newPos.x;
		y = newPos.y;
		z = newPos.z;
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
	};

	public float getX() {
		return x;
	}

	public float getY() {
		return y;
	}

	public float getZ() {
		return z;
	}

	public float getDistSquared(Point3 pos2) {
		return (x - pos2.x) * (x - pos2.x) + (y - pos2.y) * (y - pos2.y) + (z - pos2.z) * (z - pos2.z);
	}

	/*
	 * calulate the distance between the current and the specified position
	 */
	public float getDist(Point3 pos2) {
		return (float) Math.sqrt(getDistSquared(pos2));
	}

	/*
	 * calculate the norm direction vector from the current to the specified
	 * position
	 */
	public Point3 normDir(Point3 pos2) {
		float dist = getDist(pos2);
		Point3 normDir = new Point3();
		normDir.setX((pos2.x - x) / dist);
		normDir.setY((pos2.y - y) / dist);
		normDir.setZ((pos2.z - z) / dist);
		return normDir;
	}

	/*
	 * rotate the coordinates about the Z-axis by the specified amount of
	 * degrees
	 */
	public Point3 rotZ(float ang) {
		float newX = (float) (x * Math.cos(ang) - y * Math.sin(ang));
		float newY = (float) (x * Math.sin(ang) + y * Math.cos(ang));
		Point3 newPos = new Point3(newX, newY, z);
		return newPos;
	}

	/*
	 * calculate dot product
	 */
	public float dot(Point3 pos2) {
		return x * pos2.x + y * pos2.y + z * pos2.z;
	}

	public String toString() {
		return String.format("%8.2f %8.2f %8.2f", x, y, z);
	}
}
