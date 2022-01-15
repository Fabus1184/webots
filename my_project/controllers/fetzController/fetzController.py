# -*- coding: utf-8 -*-

import torch
from fetzAgent import FetzAgent
from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV
from fetzAgent import Transition

torch.autograd.set_detect_anomaly(True)

print("-----------------------")
print("CUDA INFO:")
print(f"cuda available: {torch.cuda.is_available()}")
print(f"device count: {torch.cuda.device_count()}")
print(f"device name: {torch.cuda.get_device_name()}")
print(f"device properties: {torch.cuda.get_device_properties(0)}")
print("-----------------------")


class FetzSupervisor(SupervisorCSV):
    def __init__(self):
        super().__init__()
        self.observationSpace = 3 + 3 * 100
        self.actionSpace = 3

        self.robot = None
        self.respawnRobot()
        self.messageReceived = None
        self.episodeCount = 0
        self.episodeLimit = 100000
        self.stepsPerEpisode = 20000
        self.episodeScore = 0
        self.episodeScoreList = []
        self.light_threshold = 500

        self.input_buffer = [1 for _ in range(self.observationSpace)]
        self.max_fail = 0
        self.steps = 0

    def respawnRobot(self):
        if self.robot is not None:
            self.robot.remove()

        rootNode = self.supervisor.getRoot()
        childrenField = rootNode.getField('children')
        childrenField.importMFNode(-2, "E-puck.wbo")

        self.robot = self.supervisor.getFromDef("EPUCK")

    def get_observations(self):
        self.messageReceived = self.handle_receiver()
        self.steps += 1
        if self.messageReceived is not None:
            self.input_buffer = self.input_buffer[:-3]
            self.input_buffer += [int(float(s) < self.light_threshold) for s in self.messageReceived]

        return self.input_buffer

    def get_reward(self, _):
        if self.steps > 30:
            return 1
        return 0

    def is_done(self):
        if all([bool(s) for s in self.input_buffer[-3:]]):
            self.max_fail += 1
        elif all([not bool(s) for s in self.input_buffer[-3:]]):
            self.max_fail += 1
        else:
            self.max_fail = 0

        return self.max_fail > 25

    def reset(self):
        self.input_buffer = [1 for _ in range(self.observationSpace)]
        self.steps = 0
        self.max_fail = 0
        self.respawnRobot()
        self.messageReceived = None
        return self.input_buffer

    def get_info(self):
        return None


supervisor = FetzSupervisor()
agent = FetzAgent(supervisor.observationSpace, supervisor.actionSpace)

# agent.load("scheiss-model")

while supervisor.episodeCount < supervisor.episodeLimit:
    observation = supervisor.reset()
    supervisor.episodeScore = 0

    for step in range(supervisor.stepsPerEpisode):
        selectedAction, actionProb = agent.work(observation, type_="selectAction")
        newObservation, reward, done, info = supervisor.step([selectedAction])

        trans = Transition(observation, selectedAction, actionProb, reward, newObservation)
        agent.storeTransition(trans)

        if done:
            supervisor.episodeScoreList.append(supervisor.episodeScore)
            agent.trainStep()
            break

        supervisor.episodeScore += reward
        observation = newObservation

    print("Episode #", supervisor.episodeCount, "score:", supervisor.episodeScore,
          "avg30: %.2f" % (sum(supervisor.episodeScoreList[-30:]) / 30))

    if supervisor.episodeCount % 100 == 0:
        print("SAVING MODEL")
        agent.save("scheiss-model")

    supervisor.episodeCount += 1

# # DEPLOY
# observation = supervisor.reset()
# while True:
#     selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
#     observation, _, _, _ = supervisor.step([selectedAction])
