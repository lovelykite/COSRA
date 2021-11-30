from dynamixel_sdk import *
import dx_conf

class dynamixel:   # This class handles a bundle of DXs through U2D2. Each class should be assigned to each U2D2.
    def __init__(self, device_name='/dev/ttyUSB0', protocol_version=2.0):
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(protocol_version)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(dx_conf.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, dx_conf.ADDR_GOAL_POSITION, dx_conf.LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instance for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION)

    def close_port(self):
        self.portHandler.closePort()

    def enable_torque(self, dxl_id, flag):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_TORQUE_ENABLE, flag)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("dynamixel has been successfully connected")

    def enable_led(self, dxl_id, flag):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_LED_ENABLE, flag)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("dynamixel has been successfully connected")


    def set_pos(self, dxl_id, goal_position):
        # thresholding min, max
        if goal_position < dx_conf.DXL_MINIMUM_POSITION_VALUE:
            goal_position = dx_conf.DXL_MINIMUM_POSITION_VALUE
        elif goal_position > dx_conf.DXL_MAXIMUM_POSITION_VALUE:
            goal_position = dx_conf.DXL_MAXIMUM_POSITION_VALUE

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_pos_sync(self, dxl_ids, goal_positions):
        for id, pos in zip(dxl_ids, goal_positions):
            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos)),
                                   DXL_LOBYTE(DXL_HIWORD(pos)), DXL_HIBYTE(DXL_HIWORD(pos))]

            # Add dynamixel#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(id, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % id)
                quit()

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

    def get_pos(self, dxl_id):
        dxl_current_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, dx_conf.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_current_position

    def get_pos_sync(self, dxl_ids):
        for id in dxl_ids:
            # Add parameter storage for dynamixel present position value
            dxl_addparam_result = self.groupSyncRead.addParam(id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % id)
                quit()

        # Syncread present position
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        dxl_current_positions = []
        for id in dxl_ids:
            # Check if groupsyncread data of dynamixel is available
            dxl_getdata_result = self.groupSyncRead.isAvailable(id, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                quit()

            # Get the present position value
            dxl_current_positions.append(self.groupSyncRead.getData(id, dx_conf.ADDR_PRESENT_POSITION, dx_conf.LEN_PRESENT_POSITION))

        # Clear syncread parameter storage
        self.groupSyncRead.clearParam()
        return dxl_current_positions



if __name__ == '__main__':
    # dxls = dynamixel()
    # for i in range(4):
    #     dxls.enable_torque(i, True)
    #
    # dxl_ids = [0, 1, 2, 3]
    # goal_positions = [0, 0, 0, 0]
    # dxls.set_pos_sync(dxl_ids, goal_positions)
    # time.sleep(1)
    # print (dxls.get_pos_sync(dxl_ids))

    # dxls.close_port()
    pass
