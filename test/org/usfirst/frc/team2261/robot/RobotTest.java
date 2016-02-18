package org.usfirst.frc.team2261.robot;

import static org.junit.Assert.*;

import org.junit.Ignore;
import org.junit.Test;

public class RobotTest {
	
	double[] inputarray = {-1, -.5, -.25, 1, -1, .75};
	double[] expectedOutput = {-1, -.5, -.25, 1, -1, .75};

	@Ignore ("not finished; partial test")
	public void testGetDampenedJoystickInput() {
		for (double input : inputarray ){
			for (int t = 0; t < 3; t++ ){
//				assertEquals(expectedOutput, Robot.getDampenedJoystickInput);
			}
		}
	}

}
