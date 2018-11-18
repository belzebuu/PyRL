# Articles discovered so far

## Multi-agent

### Multi-agent RL for TSC, ieee

Formulates the problem as a discounted MDP, with multiple agents.
The paper handles only independent agents. However, the cost / reward function is based on feedback from neighboring agents.

The paper focuses on average delay and uses the _VISSIM_ simulator.

### TLC by Multiagent RL Systems

Chapter from a book, which intitally establishes a basic framework, where agents are independent, and only have local information as their state. Three extensions are then made:

* Congestion information from neighboring agents taken into account
* Handling partially observability of traffic states (shouldn't be relevant, but could be based on the simulator.)
* Coordinationg the behavior between agents

Simulator used: GLD (Green Light District).

### MARLIN-ATSC (Multiagent Reinforcement Learning for Integrated Network of Adaptive Traffic Signal Controllers)

This paper touches two approaches to the multi-agent view on the problem:

* Independent mode, where each agent works independently from eachother.
* Integrated mode, where the agents are able to coordinate explicitly.

Note that due to the nature of Reinforcement Learning, implicit coordination is possible in both scenarios (but not guaranteed).
The paper focuses on average delay.
Simulator used: Paramics (micro)

### RL with Average Cost for Adaptive TSC (mentions multiagent approaches)

Primarily focus on function-approximation techniques, and compare with multiple fixed-time approaches, as well as a Q-learning agent with full state representation.
Focuses more on implementability in larger networks, at the cost of (close to) optimality.

Simulator used: GLD (Green Light District).

## Single-agent

### RL for Adaptive TSC

Machine learning project from Standford, utilizing Q-learning.
Considers only a single traffic light, hence the single agent.
Simulator used: SUMO (micro).

## In-depth

### Reinforcement Learning for True Adaptive Traffic Signal Control

Among the first attempts at utilizing reinforcement learning for adaptive traffic control, this paper explores both the single- and multi-agent approaches, however results were only represented for the single-agent approach.

#### Method(s)

* Q-learning (Off-policy TD algorithm).
* Q-Values stored in CMAC (Cerebellar Model  Articulation  Controller) -- a neural-network like data structure, in this case used more like a lookup-table than usual neural network.
* epsilon-greedy action selection, epsilon = 0.9

#### State

* In the single agent case, state was by default defined by a vector of queue lengths for each lane.
* Generalization implemented through the cmac changed this to a resolution of 50% i.e. the queue lengths of two adjacent lanes are combined.

#### Action

* In the single-agent case, the state-action space is reduced by giving the agent the choice of changing or staying with the current phase. The agent gets this choice at some point between 10 seconds into the cycle and 10 seconds before its end (the time bounds are based on the fixed-time method).
* In the multi-agent case additional information is suggested, in the form of up- and down-stream queue lengths from adjacent agents.

#### Reward

* Reward is given in the form of a penalty equal to the cumulated waiting time since last action.

#### Other

* Brings up a couple of possible ways of agent communication strategies

----------------

### Urban traffic signal control using reinforcement learning agents

#### Other

* Poorly written paper, will not use.

----------------

### Multi-agent Reinforcement Learning for Traffic Signal Control

#### Method(s)

* Q-Learning (Off-policy TD algorithm)
* UCB (Upper Confidence Bound) action selection
* epsilon-greedy action selection

----------------

### Evaluating reinforcement learning state representations for adaptive traffic signal control

#### Method(s)

* A3C by Google DeepMind (Algorithm for asynchronous deep reinforcement learning)

#### State

#### Action

#### Reward

#### Other

* Essetially does exactly what was planned for this project