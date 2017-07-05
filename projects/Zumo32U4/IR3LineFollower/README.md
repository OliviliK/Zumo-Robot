# IR3LineFollower

/**
 * Demonstrate the classic linefollower using a single sensor (front) and
 * integer PID control.  The two other Zumo line sensors are used to do 
 * sharp turns to allow the PID control to run on lower kP value.
 * 
 * The selection of tracking the left or right edge of the tape is done 
 * during robot start.
 * 
 * The robot will end after 20,000 ms or when the left and right line sensor
 * see the tape at the same time.
 */
