from PicoMotor8742Controller import PicoMotor8742Controller
from PyDAQmx import *
from PyDAQmx.DAQmxConstants import *

import time
import numpy as np

#Globals for communication between computer and motors. Entered by user.
host = '169.254.179.96'
port = 23

#Number of samples wanted. More samples means less noise but longer read time. Entered by user.
num_samples = 1000

#The length of optimization time in seconds. Entered by user.
TIMEOUT = 300

#Desired power of fiber coupling for optimization. Entered by user.
desired_power = 30

#Creates communication between computer and motors.
pmc = PicoMotor8742Controller(host, port, 1)

#These globals define the use of the ThorLabs PM100A power meter used in the creation of this program.
power_meter = 'PM100A'
number_of_rescales = 0

#Read the output of the power meter in the form of an array of outputs.
def read_output():
    read_array = np.zeros(num_samples)
    samples_read = int32() 
    task = Task()
    task.CreateAIVoltageChan('Dev3/AI0', "", DAQmx_Val_RSE, 0, 2, DAQmx_Val_Volts, None)
    task.StartTask()
    task.ReadAnalogF64(
        num_samples, 
        -1,    
        DAQmx_Val_GroupByScanNumber, 
        read_array, 
        num_samples, 
        samples_read, 
        None
    )
    return read_array

#Function used only if the power_meter = 'PM100A'
#Power converts power meter output into mW and prompts rescaling of power meter to keep track of powers of 10.
def convert_PM100A_meter_output_to_mW():
    global number_of_rescales
    output = read_output().mean()
    while output > 1.8:
        input('Increase power meter range and then press enter.')
        number_of_rescales = number_of_rescales + 1
        output = read_output().mean()
    power = (output / 2) * 0.0056 * 10 ** number_of_rescales
    return power

#Find an average of the output array. 
#Needs the power_meter string to know how to read/average specific power meter output.
def average_output():
    if power_meter == 'PM100A':
        mean_output = convert_PM100A_meter_output_to_mW()
    else:
        mean_output = read_output().mean()
    return mean_output

#Relative motion of one motor. Used in the correct_hysteresis and explore functions.
#Requires the number of motor wanted in motion and its displacement.
def move_one_motor_rel(motor, motor_displacement):
    pmc.move_axis_rel(axis=motor, displacement=motor_displacement)

#Absolute motion of one motor. Not in use currently*
#Requires the number of motor wanted in motion and its destination.
def move_one_motor_abs(motor, motor_target_position):
    pmc.move_axis_rel(axis=motor, target_position=motor_target_position)

#Relative motion of two motors. Used in the correct_hysteresis funtion.
#Requires the numbers of the motors wanted in motion and their displacements.
def move_two_motors_rel(motor1, motor2, motor1_displacement, motor2_displacement):
    pmc.move_axis_rel(axis=motor1, displacement=int(motor1_displacement))
    pmc.move_axis_rel(axis=motor2, displacement=int(motor2_displacement))

#Absolute motion of two motors. Not in use currently*
#Requires the numbers of the motors wanted in motion and their destinations.
def move_two_motors_abs(motor1, motor2, motor1_target_position, motor2_target_position):
    pmc.move_axis_abs(axis=motor1, target_position=int(motor1_target_position))
    pmc.move_axis_abs(axis=motor2, target_position=int(motor2_target_position))

#Absolute motion of four motors. Used all throughout optimize function.
#Requires the numbers of the motors wanted in motion and their displacements in the form of an array.
def move_four_motors_abs(motor1, motor2, motor3, motor4, motor_positions):
    motor1_target_position = motor_positions[0]
    motor2_target_position = motor_positions[1]
    motor3_target_position = motor_positions[2]
    motor4_target_position = motor_positions[3]
    pmc.move_axis_abs(axis=motor1, target_position=int(motor1_target_position))
    pmc.move_axis_abs(axis=motor2, target_position=int(motor2_target_position))
    pmc.move_axis_abs(axis=motor3, target_position=int(motor3_target_position))
    pmc.move_axis_abs(axis=motor4, target_position=int(motor4_target_position))

#Sets motor's current position to 0. Used in randomize_initial_simplex function to set current positon to origin.
#Requires number of motor to be set to 0.
def set_motor_home(motor):
    pmc.set_axis_home(axis=motor, home_position=0)

#Sets motor positions to 0 and randomizes the initial simplex of positions before optimization. Used all throughout optimize function.
#Requires the numbers of motors to have positions randomized and a range at which to search for positions from + to - the given range.
def randomize_initial_simplex(motor1, motor2, motor3, motor4, simplex_range):
    global simplex, output_simplex
    set_motor_home(motor1)
    set_motor_home(motor2)
    set_motor_home(motor3)
    set_motor_home(motor4)
    motors_position1 = [0, 0, 0, 0]
    read_output()
    output_position1 = average_output()
    motors_position2 = np.random.randint(low=-(simplex_range/2), high=(simplex_range/2), size=4)
    motors_position2 = motors_position2.tolist()
    move_four_motors_abs(1, 2, 3, 4, motors_position2)
    read_output()
    output_position2 = average_output()
    motors_position3 = np.random.randint(low=-(simplex_range/2), high=(simplex_range/2), size=4)
    motors_position3 = motors_position3.tolist()
    move_four_motors_abs(1, 2, 3, 4, motors_position3)
    read_output()
    output_position3 = average_output()
    motors_position4 = np.random.randint(low=-(simplex_range/2), high=(simplex_range/2), size=4)
    motors_position4 = motors_position4.tolist()
    move_four_motors_abs(1, 2, 3, 4, motors_position4)
    read_output()
    output_position4 = average_output()
    motors_position5 = np.random.randint(low=-(simplex_range/2), high=(simplex_range/2), size=4)
    motors_position5 = motors_position5.tolist()
    move_four_motors_abs(1, 2, 3, 4, motors_position5)
    read_output()
    output_position5 = average_output()
    simplex = [motors_position1, motors_position2, motors_position3, 
               motors_position4, motors_position5]
    output_simplex = [output_position1, output_position2, output_position3, 
                      output_position4, output_position5]

#Used in order function so the simplex can be sorted by outputs.
#Requires no inputted val and sorts by last element in array.
def sort_by_output(val):
    return val[-1]

#Orders simplex positions from least to greatest output. Used all throughout optimize function.
#Requires simplex and output_simplex arrays.
def order(positions, output_positions):
    simplex[0].append(output_simplex[0])
    simplex[1].append(output_simplex[1])
    simplex[2].append(output_simplex[2])
    simplex[3].append(output_simplex[3])
    simplex[-1].append(output_simplex[-1])
    simplex.sort(key=sort_by_output)
    output_simplex.sort()
    x = output_simplex[0]
    simplex[0].remove(x)
    x = output_simplex[1]
    simplex[1].remove(x)
    x = output_simplex[2]
    simplex[2].remove(x)
    x = output_simplex[3]
    simplex[3].remove(x)
    x = output_simplex[-1]
    simplex[-1].remove(x)

#Solves for the centroid of the simplex excluding the worst position. Used in elements of downhill_simplex and optimize function.
#Requires no inputs and uses whatever the current global simplex is.
def centroid():
    global centroid_position, best_position, worst_position
    number_of_motors = 4
    best_position = np.array(simplex[-1])
    worst_position = np.array(simplex[0])
    centroid_position = (best_position + np.array(simplex[3]) + 
                        np.array(simplex[2]) + np.array(simplex[1]))/(number_of_motors)

#Solves for a position reflected from the worst position. Used in elements of downhill_simplex and optimize function.
#Requires the numbers of the four motors.
def reflection(motor1, motor2, motor3, motor4):
    global reflection_position, reflection_output
    reflection_position = centroid_position + 1 * (centroid_position - worst_position)
    move_four_motors_abs(motor1, motor2, motor3, motor4, reflection_position)
    read_output()
    reflection_output = average_output()

#Solves for a position expanded from the reflected position. Used in elements of downhill_simplex and optimize function.
#Requires the numbers of the four motors.
def expansion(motor1, motor2, motor3, motor4):
    global expansion_position, expansion_output
    expansion_position = centroid_position + 2 * (reflection_position - centroid_position)
    move_four_motors_abs(motor1, motor2, motor3, motor4, expansion_position)
    read_output()
    expansion_output = average_output()

#Solves for a position contracted inside the simplex. Used in elements of downhill_simplex and optimize function.
#Requires the numbers of the four motors.
def contraction(motor1, motor2, motor3, motor4):
    global contraction_position, contraction_output
    contraction_position = centroid_position + 0.5 * (worst_position - centroid_position)
    move_four_motors_abs(motor1, motor2, motor3, motor4, contraction_position)
    read_output()
    contraction_output = average_output()

#Solves for all new simplex positions shrunk toward the current best position. Used in elements of downhill_simplex and optimize function.
#Requires the numbers of the four motors.
def shrink(motor1, motor2, motor3, motor4):
    global shrink_position1, shrink_output1, shrink_position2, shrink_output2, shrink_position3, shrink_output3, shrink_position4, shrink_output4
    shrink_position1 = best_position + 0.5 * (worst_position - best_position)
    move_four_motors_abs(motor1, motor2, motor3, motor4, shrink_position1)
    read_output()
    shrink_output1 = average_output()
    shrink_position2 = best_position + 0.5 * (np.array(simplex[1]) - best_position)
    move_four_motors_abs(motor1, motor2, motor3, motor4, shrink_position2)
    read_output()
    shrink_output2 = average_output()
    shrink_position3 = best_position + 0.5 * (np.array(simplex[2]) - best_position)
    move_four_motors_abs(motor1, motor2, motor3, motor4, shrink_position3)
    read_output()
    shrink_output3 = average_output()
    shrink_position4 = best_position + 0.5 * (np.array(simplex[3]) - best_position)
    move_four_motors_abs(motor1, motor2, motor3, motor4, shrink_position4)
    read_output()
    shrink_output4 = average_output()

#Corrects hysteresis through a process of moving two motors relatively in an output increasing direction. Used in optimize function when local max is found.
#Requires the numbers of the motors to be corrected. Corrected motors should usually the two horizontals together and two verticals together.
def correct_hysteresis(motor1, motor2):
    motor1_motion = 50
    motor2_motion = 50
    counter = 0
    read_output()
    prev_output = average_output()
    move_one_motor_rel(motor1, motor1_motion)
    read_output()
    new_output = average_output()
    if new_output < prev_output:
        motor1_motion = -motor1_motion
        move_one_motor_rel(motor1, 2*motor1_motion)
        read_output()
        new_output = average_output()
    prev_output = new_output
    move_one_motor_rel(motor2, motor2_motion)
    read_output()
    new_output = average_output()
    if new_output < prev_output:
        motor2_motion = -motor2_motion
        move_one_motor_rel(motor2, 2*motor2_motion)
        read_output()
        new_output = average_output()
    while new_output >= prev_output:
        prev_output = new_output
        move_two_motors_rel(motor1, motor2, motor1_motion, motor2_motion)
        read_output()
        new_output = average_output()
    move_two_motors_rel(motor1, motor2, -motor1_motion, -motor2_motion)

#Explores one motor in one direction 2000 steps by moving this far and working back to the original position 100 steps at a time. Used in optimize function.
#Requires the motor to be explored and the direction of exploration (1 is forward and -1 is backward).
def explore_motor(motor, direction):
    explore_step = 100
    if direction < 0:
        explore_step = -explore_step
    explore_counter = 20
    best_count = 0
    read_output()
    target_output = average_output()
    move_one_motor_rel(motor, explore_counter*explore_step)
    while explore_counter >= 0:
        explore_counter = explore_counter - 1
        move_one_motor_rel(motor, -explore_step)
        read_output()
        explore_output = average_output()
        if explore_output > target_output:
            best_count = explore_counter
            target_output = explore_output
    move_one_motor_rel(motor, explore_step*best_count)

#Optimizes four motors and finds a local maximum of power. Begins with intializing a simplex and performing downhill simplex. After three iterations of same best postion
#this function corrects for hystersis and prompts the user to decide if they would like to continue optimizing. If a low ouput is found, optimizer will explore to find
#the global peak instead. Optimization with stop if desired power is achieved.
#Requires the numbers of the motors being optimized and a desired output power in mW.
#Input the mirrors' horizontal motors as motor1 and motor3 and the mirrors' vertical motors as motor2 and motor4.
def optimize(motor1, motor2, motor3, motor4, desired_power):
    global simplex, output_simplex
    # simplex_counter = 0
    hysteresis_counter = 0
    hysteresis_corrected = False
    if power_meter == 'PM100A':
        input("Start at lowest range setting (R 5.6uW) and press enter.")
    read_output()
    final_output = average_output()
    if final_output > desired_power:
        print('Desired Power achieved. Initializing small search.')
        randomize_initial_simplex(motor1, motor2, motor3, motor4, 50)
    elif desired_power > final_output > 0.9*desired_power:
        randomize_initial_simplex(motor1, motor2, motor3, motor4, 100)
    elif 0.9*desired_power > final_output > 0.5*desired_power:
        randomize_initial_simplex(motor1, motor2, motor3, motor4, 200)
    else:
        randomize_initial_simplex(motor1, motor2, motor3, motor4, 1000)
    order(simplex, output_simplex)
    deadline = time.time() + TIMEOUT
    while deadline > time.time():
        # simplex_counter = simplex_counter + 1
        # print('Simplex Counter = ' + str(simplex_counter))
        prev_best_position = simplex[4]
        centroid()
        reflection(motor1, motor2, motor3, motor4)
        if output_simplex[1] < reflection_output <= output_simplex[-1]:
            simplex[0] = reflection_position.tolist()
            output_simplex[0] = reflection_output
        elif reflection_output > output_simplex[-1]:
            expansion(motor1, motor2, motor3, motor4)
            if reflection_output > expansion_output:
                simplex[0] = reflection_position.tolist()
                output_simplex[0] = reflection_output
            else:
                simplex[0] = expansion_position.tolist()
                output_simplex[0] = expansion_output
        elif reflection_output < output_simplex[1]:
            contraction(motor1, motor2, motor3, motor4)
            if contraction_output > output_simplex[0]:
                simplex[0] = contraction_position.tolist()
                output_simplex[0] = contraction_output                
            else:
                shrink(motor1, motor2, motor3, motor4)
                simplex[0] = shrink_position1.tolist()
                output_simplex[0] = shrink_output1
                simplex[1] = shrink_position2.tolist()
                output_simplex[1] = shrink_output2
                simplex[2] = shrink_position3.tolist()
                output_simplex[2] = shrink_output3
                simplex[3] = shrink_position4.tolist()
                output_simplex[3] = shrink_output4
        order(simplex, output_simplex)
        final_position = simplex[-1]
        final_output = output_simplex[-1]
        print('Best position = ' + str(final_position))
        print('Best output = ' + str(output_simplex[-1]) + ' mW')
        if final_output > desired_power:
            move_four_motors_abs(motor1, motor2, motor3, motor4, simplex[-1])
            print('Desired Power achieved. Initializing small search.')
            randomize_initial_simplex(motor1, motor2, motor3, motor4, 50)
        if final_position == prev_best_position:
            hysteresis_counter = hysteresis_counter + 1
            if hysteresis_counter > 2:
                hysteresis_counter = 0
                move_four_motors_abs(motor1, motor2, motor3, motor4, simplex[-1])
                print('Correcting hysteresis...')
                correct_hysteresis(motor3, motor1)
                correct_hysteresis(motor4, motor2)
                print('Local Max Achieved.')
                read_output()
                final_output = average_output()
                if final_output > desired_power:
                    print('Desired Power achieved. Initializing small search.')
                    randomize_initial_simplex(motor1, motor2, motor3, motor4, 50)
                elif desired_power > final_output > 0.9*desired_power:
                    print('Re-initializing simplex: Range=100.')
                    randomize_initial_simplex(motor1, motor2, motor3, motor4, 100)
                    order(simplex, output_simplex)
                elif 0.9*desired_power > final_output > 0.5*desired_power:
                    print('Re-initializing simplex: Range=200.')
                    randomize_initial_simplex(motor1, motor2, motor3, motor4, 200)
                    order(simplex, output_simplex)
                elif 0.5*desired_power > final_output > 0.1*desired_power:
                    print('Re-initializing simplex: Range=1000.')
                    randomize_initial_simplex(motor1, motor2, motor3, motor4, 1000)
                    order(simplex, output_simplex)
                else:
                    print('Exploring...')
                    explore = True
                    exploring_motor = 1
                    direction = 1
                    explore_counter = 0
                    while explore == True:
                        explore_counter = explore_counter + 1
                        explore_motor(exploring_motor, direction)
                        direction = -direction
                        correct_hysteresis(motor3, motor1)
                        correct_hysteresis(motor4, motor2)
                        read_output()
                        explore_output = average_output()
                        if explore_output > 10*final_output:
                            print('Explore Success.')
                            explore = False
                        if explore_counter > 1:
                            exploring_motor = exploring_motor + 1
                            explore_counter = 0
                        if exploring_motor > 4:
                            correct_hysteresis(motor3, motor1)
                            correct_hysteresis(motor4, motor2)
                            print('Explore Failed.')
                            input('Optimizer may be stuck or output is too low. Couple manually to better output and press enter to restart optimization.')
                            explore = False
                    print('Explore Output = ' + str(explore_output))
                    if explore_output > desired_power:
                        print('Desired Power achieved. Initializing small search.')
                        randomize_initial_simplex(motor1, motor2, motor3, motor4, 50)
                    elif desired_power > explore_output > 0.9*desired_power:
                        randomize_initial_simplex(motor1, motor2, motor3, motor4, 100)
                        order(simplex, output_simplex)
                    elif 0.9*desired_power > explore_output > 0.5*desired_power:
                        randomize_initial_simplex(motor1, motor2, motor3, motor4, 200)
                        order(simplex, output_simplex)
                    else:
                        randomize_initial_simplex(motor1, motor2, motor3, motor4, 1000)
                        order(simplex, output_simplex)
        else:
            hysteresis_counter = 0
    print('Optimization timed out.')
    move_four_motors_abs(motor1, motor2, motor3, motor4, simplex[-1])
    print('Correcting hysteresis...')
    correct_hysteresis(motor3, motor1)
    correct_hysteresis(motor4, motor2)
    read_output()
    final_output = average_output()
    print('Final power = ' + str(final_output) + ' mW')

if __name__ == '__main__':
    optimize(1, 2, 3, 4, desired_power)