import rospy
from dynamixel_sdk import *
import time

# Dynamixel AX-12A/12+ Setup
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
LEN_GOAL_POSITION       = 2

PROTOCOL_VERSION        = 1.0
BAUDRATE                = 1000000.00
DEVICENAME              = '/dev/ttyUSB0'

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

# DXL_IDS = [1, 2]  # Your motor IDs (for 2 motors)
DXL_IDS = [1, 2, 3, 4]  # Expanded to 4 motors
# ----------------------------- #


def enable_torque(portHandler, packetHandler, dxl_id):
    result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if result != COMM_SUCCESS:
        rospy.logerr(f"Failed to enable torque on ID {dxl_id}: {packetHandler.getTxRxResult(result)}")
        return False
    if error != 0:
        rospy.logerr(f"Error setting torque on ID {dxl_id}: {packetHandler.getRxPacketError(error)}")
        return False
    rospy.loginfo(f"Torque enabled on motor {dxl_id}")
    return True


def ping_motor(portHandler, packetHandler, dxl_id):
    model_num, result, error = packetHandler.ping(portHandler, dxl_id)
    if result == COMM_SUCCESS:
        rospy.loginfo(f"Ping successful: ID {dxl_id}, Model {model_num}")
        return True
    else:
        rospy.logwarn(f"Ping failed for ID {dxl_id}: {packetHandler.getTxRxResult(result)}")
        return False


def move_individual(portHandler, packetHandler, dxl_id, position):
    # Write goal position to single motor
    result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)
    if result != COMM_SUCCESS:
        rospy.logerr(f"Write failed for motor {dxl_id}: {packetHandler.getTxRxResult(result)}")
        return False
    if error != 0:
        rospy.logerr(f"Motor {dxl_id} error on write: {packetHandler.getRxPacketError(error)}")
        return False
    rospy.loginfo(f"Motor {dxl_id} moved to {position}")
    return True


def move_all_simultaneously(portHandler, packetHandler, positions):
    # Move all motors simultaneously
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # Prepare parameters for all motors
    for i, dxl_id in enumerate(DXL_IDS):
        param = [positions[i] & 0xFF, (positions[i] >> 8) & 0xFF]
        if not groupSyncWrite.addParam(dxl_id, param):
            rospy.logerr(f"Failed to add param for motor {dxl_id}")
    
    # Send the packet to all motors simultaneously
    groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

    rospy.loginfo(f"Sent positions: {positions}")


def main():
    rospy.init_node('multi_motor_controller_node')

    # Initialize Port & Packet handlers
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open and set port
    if not portHandler.openPort():
        rospy.logerr("Failed to open port")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        rospy.logerr("Failed to set baudrate")
        return

    # Ping and enable torque on each motor
    for dxl_id in DXL_IDS:
        if not ping_motor(portHandler, packetHandler, dxl_id):
            rospy.logerr(f"Motor {dxl_id} not responding to ping. Check wiring/power.")
        enable_torque(portHandler, packetHandler, dxl_id)

    # Test moving each motor individually
    for dxl_id in DXL_IDS:
        move_individual(portHandler, packetHandler, dxl_id, 512)
        time.sleep(1)
        move_individual(portHandler, packetHandler, dxl_id, 256)
        time.sleep(1)
    
    # -------------------------
    # After torque enable:
    ADDR_MOVING_SPEED = 32
    SPEED = 256

    for dxl_id in DXL_IDS:
        result, error = packetHandler.write2ByteTxRx(
            portHandler, dxl_id, ADDR_MOVING_SPEED, SPEED
        )
        if result != COMM_SUCCESS:
            rospy.logerr(f"Failed to set speed on {dxl_id}: {packetHandler.getTxRxResult(result)}")
        elif error != 0:
            rospy.logerr(f"Motor {dxl_id} error setting speed: {packetHandler.getRxPacketError(error)}")
        else:
            rospy.loginfo(f"Speed set to {SPEED} on motor {dxl_id}")
    # -------------------------

    # Alternate goal positions for the 4 motors
    goal_positions = [[512, 0, 512, 0], [0, 512, 0, 512]] # 2 sets of goal positions for 4 motors

    rate = rospy.Rate(0.25)  # 0.25 Hz = every 4 seconds
    i = 0
    while not rospy.is_shutdown():
        positions = goal_positions[i % 2]  # Alternate positions
        move_all_simultaneously(portHandler, packetHandler, positions)
        rospy.loginfo(f"Sent positions: {positions}")
        i += 1
        rate.sleep()

    # Disable torque on all motors
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    portHandler.closePort()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
