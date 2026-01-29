import rospy
import time
import math
from dynamixel_sdk import *

# Dynamixel AX-12A/12+ Setup
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
LEN_GOAL_POSITION       = 2

ADDR_MOVING_SPEED       = 32

PROTOCOL_VERSION        = 1.0
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

# 12 Dynamixels in X-Y-X repeating pattern
DXL_IDS = list(range(1, 13))

def enable_motor(portHandler, packetHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MOVING_SPEED, 200)



def sidewinding_positions(num_motors, t):
    """
    Sidewinding with DIFFERENT amplitudes:
    - Horizontal motors (even index) use AMP_H
    - Vertical motors (odd index) use AMP_V
    """

    AMP_H = 200
    AMP_V = 100
    FREQ = 0.3
    PHASE_SHIFT = 1.4

    positions = [0] * num_motors

    for i in range(num_motors):
        base = 512
        module_phase = (i * 1)

        if i % 2 == 0:  
            # Horizontal—larger amplitude
            pos = base + int(AMP_H * math.sin(2 * math.pi * FREQ * t + module_phase))
        else:
            # Vertical—smaller amplitude
            pos = base + int(AMP_V * math.sin(2 * math.pi * FREQ * t + module_phase + PHASE_SHIFT))

        positions[i] = max(0, min(1023, pos))

    return positions



def move_all(portHandler, packetHandler, positions):
    group = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    for dxl_id, pos in zip(DXL_IDS, positions):
        param = [pos & 0xFF, (pos >> 8) & 0xFF]
        group.addParam(dxl_id, param)

    group.txPacket()
    group.clearParam()



def main():
    rospy.init_node("snake_sidewinding_controller")

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        rospy.logerr("Failed to open port")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        rospy.logerr("Failed to set baud")
        return

    # Enable torque
    for dxl_id in DXL_IDS:
        enable_motor(portHandler, packetHandler, dxl_id)
        rospy.loginfo(f"Enabled motor {dxl_id}")

    rospy.loginfo("Starting Sidewinding Motion...")
    rate = rospy.Rate(50)  # 50 Hz smooth motion

    t0 = time.time()

    while not rospy.is_shutdown():
        t = time.time() - t0
        positions = sidewinding_positions(12, t)
        move_all(portHandler, packetHandler, positions)
        rate.sleep()

    # Disable torque when stopping
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    portHandler.closePort()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass