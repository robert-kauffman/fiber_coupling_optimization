# Automated Fiber Coupling of Two Mirrors using NewFocus Picomotors
This program automates the fiber coupling process of two mirrors with four picomotors after some power is being read by a power meter (about 0.5 uW or more).

## Required Equipment
This program requires the use of [NewFocus PicomotorTM Piezo Linear Actuators](https://www.newport.com/f/picomotor-piezo-linear-actuators). It is designed to be used by the open-loop motors. The program is also setup to use ThorLabs PM100A power meter.  To use other power meters with limited ranges, some extra programming is required. To receive analog imput into the computer, a National Instruments DAQ is also needed.

## Dependencies
This code requires python, PyDAQmx, and numpy.

The PicoMotor8742Controller library is bundled.

###Usage
To run this program, run `python fiber_coupling_optimization.py` from a terminal in the project directory. You may need to modify the script first see "Globals".

## How It Works
This program uses the Nelder-Mead Method (aka Downhill Simplex) to perform the optimzation of the four motor positions. This method is built into the optimize function. Included are a few author created functions that work to correct the picomotor hysteresis and explore the search space to defeat local maxima. More specifics about the program are as follows.

### Globals
The program requires some globals from the user for their specific study including a host IP Address, port number, number of samples taken, an optimization timeout, and a desired coupling power. These are at the top of `fiber_coupling_optimization.py`.

### Reading/Averaging Data
The read_output function makes an array of the outputs read by the power meter. The size of the array depends on the number of samples inputted by the user. The average_output function averages the array and returns a final mean output value.

### Moving Motors
There are five functions corresponding to the motion of motors. The first two are motion of just one motor in absolute displacement and relative distance. The second two are motion of two motors in absolute displacement and relative distance. The fifth motion function is the absolute motion of all four motors. This function is a bit different, for it requires an array of motor positions to be used rather than just the displacement value for each motor. The reason is cleaner code during the optimization.

###Initial Simplex
Downhill simplex needs initial positions to perform optimization. The randomize_initial_simplex function will set the current position to the origin with set_motor_home function and then randomly search over a specified space for new positions.

###Downhill Simplex
There are many steps to this method, but the first uses the order function to order the positions by their output from lowest to highest. The following functions: centroid, reflection, expansion, contraction, and shrink are all parts of the process of searching for better positions and finding the most optimal point.

###Hysteresis Correction
The Picomotors have a hardware limitation in which they cannot remember previous absolute positions. Therefore, hysteresis causes the prior absolute position in steps to be different the next time the optimizer returns to it. The correct_hysteresis function corrects the hysteresis of the best position by determining which direction each motor needs to move to get back to the maximum.

###Explore
The explore function helps the optimizer emerge from local maxima to find the global curve of the fiber coupling cost function. The function simply searches each motor in the forward and backward directions until it finds a drastic increase in power. Direction should be inputted as 1 for forward motion and -1 for backward motion.

###Optimize
This is the main function of the program and called upon to find the maximum output possible in the fiber coupling process. This function needs the numbers of the motors used and a desired power input value to be used in the case of constraining the optimizer's search when near the maximum. The horizontal motors should be entered as motor1 and motor3, while the vertical motors should be entered as motor2 and motor4. The optimizer uses all other functions to initialize search spaces, perform downhill simplex, correct hysteresis, explore for new maximum when needed, and finally end once it has reached the timeout value given by the user.

##Benefits
1. The program can perform fiber coupling from the very beginning with about 0.5 uW of power.
2. The program can used to find new maxima after a manual changing of the focusing lens position.
3. The program can be used for realignment in the morning after environmental changes affected lasers.

##Potential Modifications
Users could modify the code to be able to use other power meters than the ThorLabs PM100A. Modifications could also be made to make this program optimize more than just four motors at a time.

##Author
**Robert Kauffman** - *National Institute of Standards and Technology SURF Student 2019*

##Acknowledgements
* Chris Billington - *Assistance with coding questions and program suggestions.*
* Francisco Salces-Carcoba - *Author of PicoMotor8742Controller program.*
* Ian Spielman - *Mentor and project leader.*