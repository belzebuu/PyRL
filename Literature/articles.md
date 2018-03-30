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
