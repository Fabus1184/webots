from deepbots.robots.controllers.robot_emitter_receiver_csv import RobotEmitterReceiverCSV


class FetzRobot(RobotEmitterReceiverCSV):
    def __init__(self):
        super().__init__()

        self.gs0 = self.robot.getDevice("gs0")
        self.gs1 = self.robot.getDevice("gs1")
        self.gs2 = self.robot.getDevice("gs2")

        self.gs0.enable(self.get_timestep())
        self.gs1.enable(self.get_timestep())
        self.gs2.enable(self.get_timestep())

        self.wheel_left = self.robot.getDevice("left wheel motor")
        self.wheel_right = self.robot.getDevice("right wheel motor")

        self.wheel_left.setPosition(float("inf"))
        self.wheel_right.setPosition(float("inf"))

        self.wheel_left.setVelocity(0.0)
        self.wheel_right.setVelocity(0.0)

        self.wheel_left.setAvailableTorque(10)
        self.wheel_right.setAvailableTorque(10)

    def create_message(self):
        return [str(self.gs0.getValue()), str(self.gs1.getValue()), str(self.gs2.getValue())]

    def use_message_data(self, message):
        action = int(message[0])

        if action == 0:
            self.wheel_left.setVelocity(2.5)
            self.wheel_right.setVelocity(0)
        if action == 1:
            self.wheel_right.setVelocity(2.5)
            self.wheel_left.setVelocity(0)
        if action == 2:
            self.wheel_right.setVelocity(5)
            self.wheel_left.setVelocity(5)

        self.robot.step(self.get_timestep())

        self.wheel_left.setVelocity(0)
        self.wheel_right.setVelocity(0)


robot_controller = FetzRobot()
robot_controller.run()
del robot_controller
