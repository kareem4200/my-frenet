## Comments on latest experiments
- Data consists of the samples (x) that generated the optimal trajectory for a certain timestep and the conditions (c) which are the initial/goal states and the image of the scenario at each timestep. If a scenario has 100 timesteps, then it will have the same initial/goal state duplicated 100 times, and a 512 feature vector for the image at each timestep that was pre-encoded with frozen ResNet-18. 

- Model is an encoder of 2 hidden layers of 512 neurons and ReLU activations. Plus two other layers that map from 512 to z dimension which is 16, one layer outputs the mean and the other the log of the variance.

- Decoder is one hidden layer that maps from c dimension (6+512) + z (16) to 512 neurons. And an output layer maps 512 to dimension of x (3).

- It was trained for 20 epochs, with batch size of 128, and learning rate 1e-4. optimizer is Adam with weight decay of 1e-5. 

- CVAE loss is two components, reconstruction loss (how close are the samples ) + KL divergence (that measures the similarity between two probability distributions, approximate posterior q(z/x,c) and the actual posterior p(z/x,c) which is not known but approximated to be a normal distribution N~(0, 1)).

- Validation/Test losses might be deceiving. During training the validation loss was reasonable and did not indicate overfitting and during testing the batch (128 samples) loss was usually on the order of 0.005 (roughly). That indicated good samples reconstruction abilities on data never seen before.

- When testing with CommonRoad frenet all scenarios fail due to either kinematic feasibility of the trajectory or collisions.

- However, 3/~800 scenarios were planned successfully which shows potential for improvement.

## Problems/Recommendations for current setup

- The KL divergence term is in practice weighted by a tunable parameter beta which in my initial training was 1e-4. When printing the KL divergence term before weighting it is big (~27) and beta was too small that makes it has no contribution to the loss.

- The previous point might indicate posterior collapse (posterior becomes the prior q(z/x,c) = p(z) = N(0, 1) and z will carry no information about x therefore the decoder will reconstruct x only from c).

- The dataset included only the set of samples (t, d, v) that led to the optimal trajectory. Adding also other feasible samples for each timestep might be more helpful.

- The condition does not have any temporal information. Maybe conditioning on the timestep might be useful.

- The design choices for the number of layers, neurons, and latent space dimentionality is also a question mark but I cannot be sure that something is wrong with it.

- Maybe incorporate physics-informed losses. Borrow the feasibility checks from frenet and check it while training and penalize if samples produce infeasible trajectories. This would need generating trajectories for each reconstructed batch during training and check feasibility and collisions. Very expensive!

## Comparing Commonroad and CVAE samples

#### CommonRoad Reactive Planner Trajectory (Scenario __DEU_Lohmar-53_1_T-1__)
![CommonRoad RP Trajectory](https://raw.githubusercontent.com/kareem4200/commonroad-reactive-planner/refs/heads/cvae-dev/gifs/DEU_Lohmar-53_1_T-1_rp.gif)

#### CVAE Trajectory (Scenario __DEU_Lohmar-53_1_T-1__)
![CVAE Trajectory](https://raw.githubusercontent.com/kareem4200/commonroad-reactive-planner/refs/heads/cvae-dev/gifs/DEU_Lohmar-53_1_T-1_cvae.gif)

- CommonRoad RP sampler took ~58.66 seconds to plan this scenario (98 timesteps), while CVAE took ~65.433 seconds.

- Average time to encode 1 scenario image with ResNet-18 was 0.18 seconds. For 98 timesteps, time taken to encode images was ~17.6 seconds out of the 65.433 seconds.

- CommonRoad RP had 5, 10, 6 samples for lon_v, t, d respectively. Which makes it 5x10x6=300 trajectories to try.

- CVAE outputs 3 samples for each variable which makes possibilities are 3x3x3=27 trajectories.

- By intuition, time reduction in planning should be much more since number of trajectories to try in reduced from 300 to 27. But it is not the case?