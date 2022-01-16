import torch
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical
from torch import from_numpy, no_grad, save, load, tensor, clamp
from torch import float as torch_float
from torch import long as torch_long
from torch import min as torch_min
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
import numpy as np
from collections import namedtuple

Transition = namedtuple('Transition', ['state', 'action', 'a_log_prob', 'reward', 'next_state'])


class FetzAgent:
    def __init__(self, numberOfInputs, numberOfActorOutputs, clip_param=0.2, max_grad_norm=0.5, ppo_update_iters=5,
                 batch_size=5, gamma=0.99, actor_lr=0.001, critic_lr=0.003):
        super().__init__()

        self.clip_param = clip_param
        self.max_grad_norm = max_grad_norm
        self.ppo_update_iters = ppo_update_iters
        self.batch_size = batch_size
        self.gamma = gamma

        self.actor_net = Actor(numberOfInputs, numberOfActorOutputs).cuda()
        self.critic_net = Critic(numberOfInputs).cuda()

        self.actor_optimizer = optim.Adam(self.actor_net.parameters(), actor_lr)
        self.critic_net_optimizer = optim.Adam(self.critic_net.parameters(), critic_lr)

        self.buffer = []

    def work(self, agentInput, type_="simple"):
        agentInput = from_numpy(np.array(agentInput)).float().unsqueeze(0).cuda()

        with no_grad():
            action_prob = self.actor_net(agentInput)

        if type_ == "simple":
            output = [action_prob[0][i].data.tolist() for i in range(len(action_prob[0]))]
            return output
        elif type_ == "selectAction":
            c = Categorical(action_prob)
            action = c.sample()
            return action.item(), action_prob[:, action.item()].item()
        elif type_ == "selectActionMax":
            return np.argmax(action_prob).item(), 1.0
        else:
            raise Exception("Wrong type in agent.work(), returning input")

    def getValue(self, state):
        state = from_numpy(state)
        with no_grad():
            value = self.critic_net(state)
        return value.item()

    def save(self, path):
        save(self.actor_net.state_dict(), path + '_actor.pkl')
        save(self.critic_net.state_dict(), path + '_critic.pkl')

    def load(self, path):
        actor_state_dict = load(path + '_actor.pkl')
        critic_state_dict = load(path + '_critic.pkl')
        self.actor_net.load_state_dict(actor_state_dict)
        self.critic_net.load_state_dict(critic_state_dict)

    def storeTransition(self, transition):
        self.buffer.append(transition)

    def trainStep(self, batchSize=None):
        if batchSize is None:
            if len(self.buffer) < self.batch_size:
                return
            batchSize = self.batch_size

        state = tensor([t.state for t in self.buffer], dtype=torch_float, device="cuda")
        action = tensor([t.action for t in self.buffer], dtype=torch_long, device="cuda").view(-1, 1)
        reward = [t.reward for t in self.buffer]
        old_action_log_prob = tensor([t.a_log_prob for t in self.buffer], dtype=torch_float, device="cuda").view(-1, 1)

        R = 0
        Gt = []
        for r in reward[::-1]:
            R = r + self.gamma * R
            Gt.insert(0, R)
        Gt = tensor(Gt, dtype=torch_float, device="cuda")

        for _ in range(self.ppo_update_iters):
            for index in BatchSampler(SubsetRandomSampler(range(len(self.buffer))), batchSize, False):
                Gt_index = Gt[index].view(-1, 1)
                V = self.critic_net(state[index])
                delta = Gt_index - V
                advantage = delta.detach()

                action_prob = self.actor_net(state[index]).gather(1, action[index])

                ratio = (action_prob / old_action_log_prob[index])
                surr1 = ratio * advantage
                surr2 = clamp(ratio, 1 - self.clip_param, 1 + self.clip_param) * advantage

                action_loss = -torch_min(surr1, surr2).mean()
                self.actor_optimizer.zero_grad()
                action_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.actor_net.parameters(), self.max_grad_norm)
                self.actor_optimizer.step()

                value_loss = F.mse_loss(Gt_index, V)
                self.critic_net_optimizer.zero_grad()
                value_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.critic_net.parameters(), self.max_grad_norm)
                self.critic_net_optimizer.step()

        del self.buffer[:]


class Actor(torch.nn.Module):
    def __init__(self, numberOfInputs, numberOfOutputs):
        super(Actor, self).__init__()
        self.fc1 = torch.nn.Linear(numberOfInputs, 40, device="cuda")
        self.fc2 = torch.nn.Linear(40, 10, device="cuda")
        self.action_head = torch.nn.Linear(10, numberOfOutputs, device="cuda")

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return F.softmax(self.action_head(x), dim=1)


class Critic(torch.nn.Module):
    def __init__(self, numberOfInputs):
        super(Critic, self).__init__()
        self.fc1 = torch.nn.Linear(numberOfInputs, 40, device="cuda")
        self.fc2 = torch.nn.Linear(40, 10, device="cuda")
        self.state_value = torch.nn.Linear(10, 1, device="cuda")

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.state_value(x)
