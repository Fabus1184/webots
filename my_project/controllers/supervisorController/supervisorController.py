from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV
from PPOAgent import PPOAgent, Transition


class FetzSupervisor():
    def __init__(self):
        super().__init__()
        self.observationSpace = 3 * 150
        self.actionSpace = 3

        self.robot = None
        self.respawnRobot()
        self.messageReceived = None

        self.episodeCount = 0
        self.episodeLimit = 100000
        self.stepsPerEpisode = 20000
        self.episodeScore = 0
        self.episodeScoreList = []

        self.max_fail = 0
        self.values = [0.0 for _ in range(self.observationSpace)]

        self.light_threshold = 500

    def respawnRobot(self):
        if self.robot is not None:
            self.robot.remove()

        rootNode = self.supervisor.getRoot()
        childrenField = rootNode.getField('children')
        childrenField.importMFNode(-2, "E-puck.wbo")

        self.robot = self.supervisor.getFromDef("EPUCK")

    def get_observations(self):
        self.messageReceived = self.handle_receiver()

        if self.messageReceived is not None:
            self.values = self.values[3:]
            self.values += [float(self.messageReceived[0]) / 1000]
            self.values += [float(self.messageReceived[1]) / 1000]
            self.values += [float(self.messageReceived[2]) / 1000]

            if float(self.messageReceived[0]) < self.light_threshold and float(
                    self.messageReceived[1]) < self.light_threshold and float(
                self.messageReceived[2]) < self.light_threshold:
                self.max_fail += 2
        return self.values

    def get_reward(self):
        return 1

    def is_done(self):
        if self.messageReceived is not None:
            if float(self.messageReceived[0]) > self.light_threshold and float(
                    self.messageReceived[1]) > self.light_threshold and float(
                    self.messageReceived[2]) > self.light_threshold:
                self.max_fail += 1
            else:
                self.max_fail = 0
        return self.max_fail > 5

    def reset(self):
        self.values = [0.0 for _ in range(self.observationSpace)]
        self.max_fail = 0
        self.respawnRobot()
        # self.supervisor.simulationResetPhysics()
        self.messageReceived = None
        return self.values

    def get_info(self):
        return None


supervisor = FetzSupervisor()
agent = PPOAgent(supervisor.observationSpace, supervisor.actionSpace, use_cuda=True)

# agent.load("scheiss_model")
# agent.load("model2")

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
            agent.trainStep(batchSize=step)
            break

        supervisor.episodeScore += reward
        observation = newObservation

    if supervisor.episodeCount % 100 == 0:
        print("SAVING MODEL!")
        agent.save("model2")

    print("Episode #", supervisor.episodeCount, "score:", supervisor.episodeScore,
          " avg30 score:%2.f" % (sum(supervisor.episodeScoreList[-30:]) / len(supervisor.episodeScoreList[-30:])))
    supervisor.episodeCount += 1

# observation = supervisor.reset()
# while True:
#     selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
#     observation, _, _, _ = supervisor.step([selectedAction])
