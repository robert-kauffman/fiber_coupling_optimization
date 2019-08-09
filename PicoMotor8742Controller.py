# Picomotor Control v1.0

author = '__pacosalces__'

import telnetlib
import time

CR = b"\n"
LF = b"\r"

def telnet_8742(command):
    """ Assemble raw commands for 8742 
    picomotor controller and iterface to telnet
    server """
    def wrapped(func):
        def telnet_interface(self, *args, **kwargs):
            try:
                value = func(self, *args, **kwargs)
                self.connect_to_telnet()
                if "axis" in kwargs.keys():
                    axis = str(kwargs['axis'])
                else:
                    axis = ""
                if value is not None:
                    value = str(value)
                else:
                    value = ""
                line = (axis + command + value).encode()
                self.send_to_telnet(line)
                if b"?" in line:
                    data = self.receive_from_telnet()
                else:
                    data = None
                self.close_connection()
                return data
            except Exception as error:
                print(error)
                print("\n More info: \n")
                print("Command: " + func.__name__ + "()")
                print(func.__doc__)
                return None
        return telnet_interface
    return wrapped

class PicoMotor8742Controller(object):
           
    def __init__(self, host='localhost', port=23, timeout=0.005):
        self.host = host
        self.port = port
        self.timeout = timeout
        
    def connect_to_telnet(self):
        self.tn = telnetlib.Telnet(self.host, self.port, self.timeout)
        time.sleep(0.5)

    def send_to_telnet(self, line):
        self.tn.write(line + LF + CR)
        time.sleep(0.01)

    def receive_from_telnet(self):
        response = self.tn.read_until(LF + CR)
        time.sleep(1)
        return response.decode()

    def close_connection(self):
        self.tn.close()

    def check_value(self, value, allowed, message):
        if value not in allowed:
            raise ValueError(message)

    def check_axis(self, axis):
        self.check_value(axis, range(1, 5), "Invalid axis")

    @telnet_8742("*IDN?")
    def identity(self):
        """
        This query will cause the instrument to return a 
        unique identification string. This similar to the
        Version (VE) command but provides more information. 
        In response to this command the controller replies 
        with company name, product model name, firmware version
        number, firmware build date, and controller serial 
        number. No two controllers share the same model name 
        and serial numbers, therefore this information can 
        be used to uniquely identify a specific controller.
        """

    @telnet_8742("*RCL")
    def recall_setting(self, parameter_bin):
        """ 
        This command restores the controller working 
        parameters in parameter_bin [0, 1] from values 
        saved in its nonvolatile memory. It is useful when, 
        for example, the user has been exploring and changing
        parameters (e.g., velocity) but then chooses to reload 
        from previously stored, qualified settings. Note that 
        “*RCL 0” command just restores the working parameters 
        to factory default settings. It does not change the 
        settings saved in EEPROM. 

        The parameters that can be restored are:
            1. Hostname (see HOSTNAME command)
            2. IP Mode (see IPMODE command)
            3. IP Address (see IPADDRESS command)
            4. Subnet mask address (see NETMASK command)
            5. Gateway address (see GATEWAY command)
            6. Configuration register (see ZZ command)
            7. Motor type (see QM command)
            8. Desired Velocity (see VA command)
            9. Desired Acceleration (see AC command) 
        """
        self.check_value(parameter_bin, list([0, 1]), "Invalid parameter bin")
        return parameter_bin

    @telnet_8742("*RST")
    def reset_instrument(self):
        """
        This command performs a “soft” reset or reboot of the 
        controller CPU. Upon restart the controller reloads parameters 
        (e.g., velocity and acceleration) last saved in non-volatile
        memory. Note that upon executing this command, USB and Ethernet 
        communication will be interrupted for a few seconds while the 
        controller re-initializes. Ethernet communication may be 
        significantly delayed (~30 seconds) in reconnecting depending on 
        connection mode (Peer-to-peer, static or dynamic IP mode) as the 
        PC and controller are negotiating TCP/IP communication.
        """

    @telnet_8742("AB")
    def abort_motion(self):
        """ 
        This command is used to instantaneously stop any motion 
        that is in progress. Motion is stopped abruptly. For stop 
        with deceleration see ST command which uses programmable
        acceleration/deceleration setting.
        """

    @telnet_8742("AC")
    def set_axis_acceleration(self, axis, acceleration):
        """ 
        This command is used to set the acceleration value for 
        an axis. The acceleration setting specified will not have any 
        effect on a move that is already in progress. If this command
        is issued when an axis’ motion is in progress, the controller 
        will accept the new value but it will use it for subsequent 
        moves only.

        ::Argument::        ::Range::       ::Description::
            axis             1 to 4           Axis number
        acceleration         1 to 200000      Acceleration (steps/sec2). 
                                              Default = 100000 steps/sec2
        """
        self.check_axis(axis)
        self.check_value(abs(acceleration), range(200001), "Invalid acceleration")
        return acceleration

    @telnet_8742("AC?")
    def query_axis_acceleration(self, axis):
        """
        This command is used to query the acceleration
        value for an axis.
        """
        self.check_axis(axis)

    @telnet_8742("DH")
    def set_axis_home(self, axis, home_position):
        """ 
        This command is used to define the “home” position for an 
        axis. The home position is set to 0 if this command is issued 
        without "home_position" value. Upon receipt of this command, the
        controller will set the present position to the specified home 
        position. The move to absolute position command (PA) uses the 
        “home” position as reference point for moves.

        ::Argument::            ::Range::           ::Description::
            axis                 1 to 4               Axis number
        home_position  -2147483648 to +2147483647     Home position (steps). 
                                                      Default = 0 steps
        """
        self.check_axis(axis)
        self.check_value(home_position, range(2147483648), "Invalid home position")
        return home_position

    @telnet_8742("DH?")
    def query_axis_home(self, axis):
        """
        This command is used to query the home position 
        value for an axis
        """
        self.check_axis(axis)

    @telnet_8742("MC")
    def motor_check(self):
        """ 
        This command scans for motors connected to the controller, 
        and sets the motor type based on its findings. If the piezo motor 
        is found to be type ‘Tiny’ then velocity (VA) setting is automatically 
        reduced to 1750 if previously set above 1750. To accomplish this task, 
        the controller commands each axis to make a one-step move in the negative
        direction followed by a similar step in the positive direction. This 
        process is repeated for all the four axes starting with the first one. 
        If this command is issued when an axis is moving, the controller will 
        generate “MOTION IN PROGRESS” error message.
        NOTE: Motor type and velocity changes are not automatically saved to 
        non-volatile memory. Issue the Save (SM) command after motor check 
        to save all changes. 
        """

    @telnet_8742("MD?")
    def query_axis_motion(self, axis):
        """
        This command is used to query the motion status for an axis.
        """
        self.check_axis(axis)

    @telnet_8742("MV")
    def move_axis_indefinitely(self, axis, direction):
        """
        This command is used to move an axis indefinitely. If this command 
        is issued when an axis’ motion is in progress, the controller will ignore 
        this command and generate “MOTION IN PROGRESS” error message. Issue a Stop 
        (ST) or Abort (AB) motion command to terminate motion initiated by MV.

        ::Argument::            ::Range::           ::Description::
            axis                 1 to 4               Axis number
          direction            "+" or "-"           Desired direction.
        """                 
        self.check_axis(axis)
        self.check_value(direction, list(["+", "-"]), "Invalid direction")
        return direction

    @telnet_8742("PA")
    def move_axis_abs(self, axis, target_position):
        """ 
        This command is used to move an axis to a desired target (absolute) 
        position relative to the home position defined by DH command. Note that 
        DH is automatically set to 0 after system reset or a power cycle. If this 
        command is issued when an axis’ motion is in progress, the controller will 
        ignore this command and generate “MOTION IN PROGRESS” error message. The 
        direction of motion and number of steps needed to complete the motion will 
        depend on where the motor count is presently at before the command is issued.
        Issue a Stop (ST) or Abort (AB) motion command to terminate motion initiated 
        by PA.

         ::Argument::            ::Range::              ::Description::
             axis                 1 to 4                  Axis number
        target_position  -2147483648 to +2147483647    Absolute position (steps). 
                                                        Default = 0 steps
        """
        self.check_axis(axis)
        self.check_value(abs(target_position), range(2147483648), "Invalid target position")
        return target_position

    @telnet_8742("PA?")
    def query_axis_abs(self, axis):
        """
        This command is used to query the target position 
        of an axis.
        """
        self.check_axis(axis)

    @telnet_8742("PR")
    def move_axis_rel(self, axis, displacement):
        """
        This command is used to move an axis by a desired relative distance. 
        If this command is issued when an axis’ motion is in progress, the 
        controller will ignore this command and generate “MOTION IN PROGRESS” error 
        message. Issue a Stop (ST) or Abort (AB) motion command to terminate 
        motion initiated by PR.

        ::Argument::            ::Range::              ::Description::
             axis                 1 to 4                  Axis number
         displacement  -2147483648 to +2147483647      Relative distance (steps). 
                                                        Default = 0 steps
        """
        self.check_axis(axis)
        self.check_value(abs(displacement), range(2147483648), "Invalid displacement")
        return displacement

    @telnet_8742("PR?")
    def query_axis_rel(self, axis):
        """
        This command is used to query the target position of an axis.
        """
        self.check_axis(axis)

    @telnet_8742("QM")
    def set_motor_type(self, axis, motor_type):
        """
        This command is used to manually set the motor type of an axis. 
        Send the Motors Check (MC) command to have the controller determine 
        what motors (if any) are connected. Note that for motor type ‘Tiny’, 
        velocity should not exceed 1750 step/sec. To save the setting to 
        non-volatile memory, issue the Save (SM) command. Note that the controller
        may change this setting if auto motor detection is enabled by setting bit 
        number 0 in the configuration register to 0 (default) wit ZZ command. 
        When auto motor detection is enabled the controller checks motor presence 
        and type automatically during all moves and updates QM status accordingly.

        ::Argument::            ::Range::              ::Description::
             axis                 1 to 4                  Axis number
          motor_type                0                   No motor connected 
                                    1                   Unknown motor type
                                    2                     'Tiny motor'
                                    3                   'Standard motor'
        """
        self.check_axis(axis)
        self.check_value(motor_type, range(4), "Invalid motor type")
        return motor_type

    @telnet_8742("QM?")
    def query_motor_type(self, axis):
        """
        This command is used to query the motor type of an axis. It is important 
        to note that the QM? command simply reports the present motor typesetting 
        in memory. It does not perform a check to determine whether the setting 
        is still valid or corresponds with the motor connected at that instant.
        If motors have been removed and reconnected to different controller 
        channels or if this is the first time, connecting this system then 
        issuing the Motor Check (MC) command is recommended. This will ensure 
        an accurate QM? command response. 
        """
        self.check_axis(axis)

    @telnet_8742("RS")
    def reset_controller(self):
        """
        This command performs a “soft” reset or reboot of the controller CPU. 
        Upon restart the controller reloads parameters (e.g., velocity and 
        acceleration) last saved in non-volatile memory and sets Home (DH) position 
        to 0. Note that upon executing this command, USB and Ethernet communication 
        will be interrupted for a few seconds while the controller re-initializes.
        Ethernet communication may be significantly delayed (~30 seconds) in 
        reconnecting depending on connectionmode (Peer-to-peer, static or dynamic 
        IP mode) as the PC and controller are negotiating TCP/IP communication.
        """

    @telnet_8742("SA")
    def set_address(self, address):
        """
        This command is used to set the address of a controller. This command 
        is useful when communicating with controllers on an RS-485 network, where
        all controllers on the network must have unique addresses. The default 
        controller address is 1.

        ::Argument::            ::Range::              ::Description::
           address               1 to 31              Controller address
                                                        Default is 1
        """
        self.check_value(address, range(32), "Invalid address")
        return address

    @telnet_8742("SA?")
    def query_address(self):
        """
        This command is used to query the controller’s address.
        """

    @telnet_8742("SC")
    def set_scan_option(self, option):
        """
        This command is used to initiate scan of controllers on RS-485 network. 
        When a master controller receives this command, it scans the RS-485 
        network for all the slave controllers connected to it. If option = 0, the 
        master controller scans the network but does not resolve any address 
        conflicts. If option = 1, the master controller scans the network and 
        resolves address conflicts, if any. This option preserves the non-conflicting 
        addresses and reassigns the conflicting addresses starting with the lowest 
        available address. For example, during an initial scan, if the master 
        controller determines that there are unique controllers at addresses 1,2, 
        and 7 and more than one controller at address 23, this option will reassign 
        only the controllers with address conflict at 23; the controllers with 
        addresses 1,2, and 7 will remain untouched. In this case, after conflict 
        resolution, the final controller addresses might be 1,2,3,7, and 23 if the 
        master determines that there are two (2) controllers initially at address 23.
        If option = 2, the master controller reassigns the addresses of all 
        controllers on the network in a sequential order starting with master 
        controller set to address 1. In the example mentioned above, after 
        reassignment of addresses, the final controller addresses will be 1,2,3,
        4, and 5. 
        """

        self.check_value(option, range(3), "Invalid scan option")

    @telnet_8742("SC?")
    def query_scan_option(self):
        """
        This command is used to query the list of all 
        controllers on an RS-485 network. 
        Returns a 32-bit value:

        ::Bit::  ::Value::                   ::Description::
           0         0       The scan process did not find any address conflicts
           0         1       The scan process found at least one address conflicts
           1         0       There is no controller with address 1 on the network
           1         1       There is a controller with address 1 on the network
           31        0       There is no controller with address 31 on the network
           31        1       There is a controller with address 31 on the network
        
        Bits 1—31 are one-to-one mapped to controller 
        addresses 1—31. The bit value is set to 1 only when 
        there are no conflicts with that address. For example, 
        if the master controller determines that there are 
        unique controllers at addresses 1,2, and 7 and more than 
        one controller at address 23, this query will return
        135. The binary representation of 135 is 10000111. 
        Bit #0 = 1 implies that the scan found at lease one 
        address conflict during last scan. Bit #1,2, 7 = 1 implies 
        that the scan found controllers with addresses 1,2, and 7 
        that do not conflict with any other controller. 
        """

    @telnet_8742("SD?")
    def query_scan_status(self):
        """
        This command is used to query the scan status.
        """

    @telnet_8742("SM")
    def save_current_settings(self):
        """
        This command saves the controller settings in its non-volatile memory. 
        The controller restores or reloads these settings to working registers 
        automatically after system reset or it reboots. The Purge (XX) command 
        is used to clear non-volatile memory and restore to factory settings. 
        Note that the SM saves parameters for all motors.

        The SM command saves the following settings:
            1. Hostname (see HOSTNAME command)
            2. IP Mode (see IPMODE command)
            3. IP Address (see IPADDRESS command)
            4. Subnet mask address (see NETMASK command)
            5. Gateway address (see GATEWAY command)
            6. Configuration register (see ZZ command)
            7. Motor type (see QM command)
            8. Desired Velocity (see VA command)
            9. Desired Acceleration (see AC command)
        """

    @telnet_8742("ST")
    def stop_axis_motion(self, axis):
        """ 
        This command is used to stop the motion of an axis. The controller 
        uses acceleration specified using AC command to stop motion. If no axis 
        number is specified, the controller stops the axis that is currently 
        moving. Use Abort (AB) command to abruptly stop motion without 
        deceleration.
        """
        self.check_axis(axis)

    @telnet_8742("TB?")
    def query_error_state(self):
        """
        This command is used to read the error code, and the associated message.
        The error code is one numerical value up to three(3) digits long. 
        (see Appendix for complete listing) In general, non-axis specific errors 
        numbers range from 0-99. Axis-1 specific errors range from 100-199, 
        Axis-2 errors range from 200-299 and so on. The message is a description 
        of the error associated with it. All arguments are separated by commas.
        Note: Errors are maintained in a FIFO buffer ten(10) elements deep. When an
        error is read using TB or TE, the controller returns the last error that 
        occurred and the error buffer is cleared by one(1) element. This means that 
        an error can be read only once, with either command.
        """

    @telnet_8742("TE?")
    def query_error_code(self):
        """
        This command is used to read the error code. The error code is one 
        numerical value up to three(3) digits long. (see Appendix for complete 
        listing) In general, non-axis specific errors numbers range from 0-99. 
        Axis-1 specific errors range from 100-199, Axis-2 errors range from 
        200-299 and so on. Note: Errors are maintained in a FIFO buffer ten(10) 
        elements deep. When an error is read using TB or TE, the controller 
        returns the last error that occurred and the error buffer is cleared by 
        one(1) element. This means that an error can be read only once, with 
        either command.
        """

    @telnet_8742("TP?")
    def query_axis_position(self, axis):
        """
        This command is used to query the actual position of an axis. 
        The actual position represents the internal number of steps made 
        by the controller relative to its position when controller was 
        powered ON or a system reset occurred or Home (DH) command was 
        received. Note that the real or physical position of the actuator/motor 
        may differ as a function of mechanical precision and inherent open-loop 
        positioning inaccuracies.
        """
        self.check_axis(axis)

    @telnet_8742("VA")
    def set_axis_velocity(self, axis, velocity):
        """
        This command is used to set the velocity value for an axis. The 
        velocity setting specified will not have any effect on a move that 
        is already in progress. If this command is issued when an axis’ motion 
        is in progress, the controller will accept the new value but it will
        use it for subsequent moves only. The maximum velocity for a ‘Standard’ 
        Picomotor is 2000 steps/sec, while the same for a ‘Tiny’ Picomotor is 
        1750 steps/sec.

        ::Argument::            ::Range::              ::Description::
           axis                  1 to 4                  Motor Axis
         velocity               1 to 2000            Velocity (steps/sec)
                                                    Default is 2000 steps/sec
        """
        self.check_axis(axis)
        self.check_value(velocity, range(2001), "Invalid velocity")
        return velocity

    @telnet_8742("VA?")
    def query_axis_velocity(self, axis):
        """
        This command is used to query the velocity value for an axis.
        """
        self.check_axis(axis)

    @telnet_8742("VE?")
    def query_controller_firmware(self):
        """
        This command is used to query the controller model number and 
        firmware version. To query product serial number information see 
        *IDN command.
        """

    @telnet_8742("XX")
    def purge_current_settings(self):
        """
        This command is used to purge all user settings in the controller 
        non-volatile memory and restore them to factory default settings.
        The following parameters are affected by this command:

            1. Hostname (see HOSTNAME command)
            2. IP Mode (see IPMODE command)
            3. IP Address (see IPADDRESS command)
            4. Subnet mask address (see NETMASK command)
            5. Gateway address (see GATEWAY command)
            6. Configuration register (see ZZ command)
            7. Motor type (see QM command)
            8. Desired Velocity (see VA command)
            9. Desired Acceleration (see AC command)
        """

    @telnet_8742("ZZ")
    def set_global_configuration(self, config_bit):
        """
        This command is used to configure the default behavior of some 
        of the controller’s features. It is typically followed with an 
        SM (Save to Memory) command.

        ::Bit#::    ::Value::                ::Description::
          *0            0          Perform auto motor detection. 
                                   Check and set motor type automatically 
                                   when commanded to move.
           0            1          Do not perform auto motor detection on move
          *1            0          Do not scan for motors connected to controllers 
                                   upon reboot(Performs ‘MC’ command upon power-up, 
                                   reset or reboot)
           1            1          Scan for motors connected to controller upon 
                                   power-up or reset
        """
        supported_options = list(["*0", "0", "*1", "1"])
        self.check_value(config_bit, supported_options, "Invalid configuration bit")

    @telnet_8742("ZZ?")
    def query_global_configuration(self):
        """
        This command is used to query the configuration register setting.
        """


if __name__ == "__main__":
    dev = Picomotor8742Controller(host="169.254.187.22", port=23, timeout=0.01)
    print(dev.query_motor_type(axis=3))

