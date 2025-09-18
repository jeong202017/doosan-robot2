import rclpy
from rclpy.node import Node

from dsr_msgs2.srv import MoveJoint, GetCurrentPosj

import time

ROBOT_NS = '/dsr01'  # 네임스페이스는 bringup 옵션(model)에 따라 다름

class DoosanJointController(Node):
    def __init__(self):
        super().__init__('doosan_joint_controller')

        # Service clients
        self.movej_client = self.create_client(MoveJoint, f'{ROBOT_NS}/motion/move_joint')
        self.get_posj_client = self.create_client(GetCurrentPosj, f'{ROBOT_NS}/aux_control/get_current_posj')

        while not self.movej_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_joint service...')
        while not self.get_posj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_current_posj service...')

    def get_current_joints(self):
        req = GetCurrentPosj.Request()
        future = self.get_posj_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            joints = list(future.result().pos)
            self.get_logger().info(f'[GET] Current Joint Values: {joints}')
            return joints
        else:
            self.get_logger().error('Failed to get current joints.')
            return None

    def move_to_joints(self, target_joints, vel=30.0, acc=30.0):
        req = MoveJoint.Request()
        req.pos = target_joints
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.sync_type = 0  # 0: sync, 1: async
        req.mode = 0       # 기본값
        req.radius = 0.0
        req.blend_type = 0

        future = self.movej_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'[SET] move_joint success: {target_joints}')
            return True
        else:
            self.get_logger().error('Failed to move joints.')
            return False

def main(args=None):
    rclpy.init(args=args)
    controller = DoosanJointController()
    # 1. 현재 조인트 값 출력
    joints = controller.get_current_joints()
    # 2. 이동할 타겟 조인트 각도 지정(단위: deg)
    target = [0.0, -45.0, 90.0, 0.0, 45.0, 0.0]  # h2017은 6축
    # 3. 조인트 이동
    controller.move_to_joints(target)
    time.sleep(2)
    # 4. 이동 후 조인트 값 다시 출력
    controller.get_current_joints()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()