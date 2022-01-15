# -*- coding: utf-8 -*-

import torch
import datetime
import torchinfo
from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV
from PPOAgent import Transition

torch.autograd.set_detect_anomaly(True)

print("-----------------------")
print("CUDA INFO:")
print(f"cuda available: {torch.cuda.is_available()}")
print(f"device count: {torch.cuda.device_count()}")
print(f"device name: {torch.cuda.get_device_name()}")
print(f"device properties: {torch.cuda.get_device_properties(0)}")
print("-----------------------")


class FetzNetz(torch.nn.Module):
    def __init__(self, n_in, n_out, n_hidden, n_layers):
        super(FetzNetz, self).__init__()

        self.fc1 = torch.nn.Linear(n_in, 40, device="cuda")
        self.rnn = torch.nn.RNN(input_size=40, hidden_size=n_hidden, num_layers=n_layers, nonlinearity="relu",
                                device="cuda")
        self.fc_out = torch.nn.Linear(n_hidden, n_out, device="cuda")

        self.n_hidden = n_hidden
        self.n_layers = n_layers

    def forward(self, x, h_state):
        if h_state is None:
            h_state = torch.zeros(size=(self.n_layers, 1, self.n_hidden), device="cuda")

        out = torch.nn.functional.relu(self.fc1(x))

        out, hidden = self.rnn(out, h_state)
        out = torch.nn.functional.relu(out.contiguous().view(-1, self.n_hidden))

        out = torch.nn.functional.softmax(self.fc_out(out), dim=1)

        return out, hidden


class KritikNetz(torch.nn.Module):
    def __init__(self, n_in, n_out, n_hidden, n_layers):
        super(KritikNetz, self).__init__()

        self.critic_fc1 = torch.nn.Linear(n_in, 30, device="cuda")
        self.critic_rnn = torch.nn.RNN(input_size=30, hidden_size=n_hidden, num_layers=n_layers,
                                       nonlinearity="relu",
                                       device="cuda")
        self.critic_fc_out = torch.nn.Linear(n_hidden, 1, device="cuda")

        self.n_hidden = n_hidden
        self.n_layers = n_layers

    def forward(self, x, h_state):
        if h_state is None:
            h_state = torch.zeros(size=(self.n_layers, 1, self.n_hidden), device="cuda")

        out = torch.nn.functional.relu(self.critic_fc1(x))

        out, hidden = self.critic_rnn(out, h_state)
        out = torch.nn.functional.relu(out.contiguous().view(-1, self.n_hidden))

        out = torch.nn.functional.softmax(self.critic_fc_out(out), dim=1)

        return out, hidden


class FetzSupervisor(SupervisorCSV):
    def __init__(self):
        super().__init__()
        self.observationSpace = 3
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
        self.light_threshold = 500

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
        if self.messageReceived is not None:
            self.steps += 1
            if all([float(s) > self.light_threshold for s in self.messageReceived]):
                self.max_fail += 2
            return [float(s) / 1000 for s in self.messageReceived]
        return [0.0 for _ in range(self.observationSpace)]

    def get_reward(self, _):
        if self.steps > 20:
            return 1
        return 0

    def is_done(self):
        if self.messageReceived is not None:
            if all([float(s) > self.light_threshold for s in self.messageReceived]):
                self.max_fail += 1
            else:
                self.max_fail = 0
        return self.max_fail > 15

    def reset(self):
        self.steps = 0
        self.max_fail = 0
        self.respawnRobot()
        self.messageReceived = None
        return [0.0 for _ in range(self.observationSpace)]

    def get_info(self):
        return None


class FetzAgent:
    def __init__(self, n_in, n_out, n_hidden, n_layers):
        self.fn = FetzNetz(n_in, n_out, n_hidden, n_layers)
        self.kn = KritikNetz(n_in, n_out, n_hidden, n_layers)

        self.actor_optimizer = torch.optim.Adam(self.fn.parameters(), 0.001)
        self.critic_net_optimizer = torch.optim.Adam(self.kn.parameters(), 0.001)

        self.buffer = []

        self.gamma = 0.99
        self.ppo_update_iters = 5
        self.clip_param = 0.2
        self.max_grad_norm = 0.5

    def work(self, input_data, h_state):
        tensor = torch.cuda.FloatTensor([[input_data]])
        out, new_hidden_state = self.fn(tensor, h_state)
        return out, new_hidden_state

    def getValue(self, state):
        pass

    def save(self, path):
        torch.save(self.fn.state_dict(), path + "_actor")
        torch.save(self.kn.state_dict(), path + "_critic")
        pass

    def load(self, path):
        self.fn.load_state_dict(torch.load(path + "_actor"))
        self.fn.eval()

        self.kn.load_state_dict(torch.load(path + "_critic"))
        self.kn.eval()
        pass

    def storeTransition(self, transition):
        self.buffer.append(transition)
        pass

    def trainStep(self):
        state = torch.tensor([t.state for t in self.buffer], device="cuda")
        hidden = torch.tensor([t.hidden.detach()[0][0].tolist() for t in self.buffer], device="cuda")

        # dim2 = state.shape[0]
        # state = torch.reshape(state, (1, dim2, 3))

        a = torch.tensor([t.action for t in self.buffer], device="cuda").view(-1, 1)
        r = [t.reward for t in self.buffer]
        old_action_log_prob = torch.tensor([t.a_log_prob for t in self.buffer], device="cuda").view(-1, 1)

        R = 0
        Gt = []
        for r in r[::-1]:
            R = r + self.gamma * R
            Gt.insert(0, R)
        Gt = torch.tensor(Gt, device="cuda")

        critic_hidden = torch.zeros(1, 1, 10, device="cuda")

        for _ in range(self.ppo_update_iters):
            start = datetime.datetime.now()
            for index in torch.utils.data.sampler.BatchSampler(torch.utils.data.sampler.SubsetRandomSampler(
                    range(len(self.buffer))), 1, False):

                Gt_index = Gt[index].view(-1, 1)
                V, critic_hidden = self.kn(state[index].view(1, 1, 3), critic_hidden)

                critic_hidden = critic_hidden.detach()

                delta = Gt_index - V
                advantage = delta.detach()

                action_prob, actor_hidden = self.fn(state[index].view(1, 1, 3), hidden[index].view(1, 1, 10))
                action_prob = action_prob.gather(1, a[index])

                ratio = (action_prob / old_action_log_prob[index])
                surr1 = ratio * advantage
                surr2 = torch.clamp(ratio, 1 - self.clip_param, 1 + self.clip_param) * advantage

                self.critic_net_optimizer.zero_grad()
                self.actor_optimizer.zero_grad()

                action_loss = -torch.min(surr1, surr2).mean()  # MAX->MIN descent
                value_loss = torch.nn.functional.mse_loss(Gt_index, V)

                action_loss.backward()
                value_loss.backward()

                torch.nn.utils.clip_grad_norm_(self.kn.parameters(), self.max_grad_norm)
                torch.nn.utils.clip_grad_norm_(self.fn.parameters(), self.max_grad_norm)

                self.actor_optimizer.step()
                self.critic_net_optimizer.step()
            print(datetime.datetime.now() - start)

        del self.buffer[:]


supervisor = FetzSupervisor()
agent = FetzAgent(supervisor.observationSpace, supervisor.actionSpace, 10, 1)

print(torchinfo.summary(agent.fn))
print(torchinfo.summary(agent.kn))

# agent.load("scheisse")

while supervisor.episodeCount < supervisor.episodeLimit:
    observation = supervisor.reset()
    supervisor.episodeScore = 0
    hidden_state = None

    for step in range(supervisor.stepsPerEpisode):
        action, hidden_state = agent.work(observation, hidden_state)

        selectedAction = torch.distributions.Categorical(action).sample()

        # action = action[0].tolist()
        # selectedAction = action.index(max(action))

        selectedAction = selectedAction.item()
        newObservation, reward, done, info = supervisor.step([selectedAction])

        prob = action.data[0][selectedAction].item()
        trans = Transition(observation, hidden_state, selectedAction, prob, reward, newObservation)
        agent.storeTransition(trans)

        if done:
            supervisor.episodeScoreList.append(supervisor.episodeScore)
            agent.trainStep()
            break

        supervisor.episodeScore += reward
        observation = newObservation

    if supervisor.episodeCount % 100 == 0:
        print("SAVING MODEL!")
        agent.save("scheisse")

    print("Episode #", supervisor.episodeCount, "score:", supervisor.episodeScore,
          " avg30 score:%2.f" % (sum(supervisor.episodeScoreList[-30:]) / len(supervisor.episodeScoreList[-30:])))

    supervisor.episodeCount += 1

# observation = supervisor.reset()
# while True:
#     selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
#     observation, _, _, _ = supervisor.step([selectedAction])
