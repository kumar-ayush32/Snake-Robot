import rospy
from dynamixel_sdk import *
import time

# Dynamixel AX-12A/12+ Setup
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
LEN_GOAL_POSITION       = 2

PROTOCOL_VERSION        = 1.0
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

DXL_IDS = [23, 24]



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
    result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)
    if result != COMM_SUCCESS:
        rospy.logerr(f"Write failed for motor {dxl_id}: {packetHandler.getTxRxResult(result)}")
        return False
    if error != 0:
        rospy.logerr(f"Motor {dxl_id} error on write: {packetHandler.getRxPacketError(error)}")
        return False
    rospy.loginfo(f"Motor {dxl_id} moved to {position}")
    return True


def main():
    rospy.init_node('dual_motor_controller_node')

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        rospy.logerr("Failed to open port")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        rospy.logerr("Failed to set baudrate")
        return

    for dxl_id in DXL_IDS:
        if not ping_motor(portHandler, packetHandler, dxl_id):
            rospy.logerr(f"Motor {dxl_id} not responding to ping. Check wiring/power.")
        enable_torque(portHandler, packetHandler, dxl_id)

    for dxl_id in DXL_IDS:
        move_individual(portHandler, packetHandler, dxl_id, 512)
        time.sleep(1)
        move_individual(portHandler, packetHandler, dxl_id, 256)
        time.sleep(1)
    
    # After torque enable:
    ADDR_MOVING_SPEED = 32
    SPEED = 300

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

    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # Alternate goal positions
    goal_positions = [[256, 768], [768, 256]]

    rate = rospy.Rate(0.5)  # 0.5 Hz = every 2 seconds
    i = 0
    while not rospy.is_shutdown():
        pos1 = goal_positions[i % 2][0]
        pos2 = goal_positions[i % 2][1]

        # Convert to bytes
        param1 = [pos1 & 0xFF, (pos1 >> 8) & 0xFF]
        param2 = [pos2 & 0xFF, (pos2 >> 8) & 0xFF]

        # Adding parameters with checks
        if not groupSyncWrite.addParam(DXL_IDS[0], param1):
            rospy.logerr(f"Failed to add param for motor {DXL_IDS[0]}")
        if not groupSyncWrite.addParam(DXL_IDS[1], param2):
            rospy.logerr(f"Failed to add param for motor {DXL_IDS[1]}")

        # Transmit
        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()

        rospy.loginfo(f"Sent positions - Motor {DXL_IDS[0]}: {pos1}, Motor {DXL_IDS[1]}: {pos2}")
        i += 1
        rate.sleep()

    # Disable torque
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    portHandler.closePort()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
