# Cobotic AI
The biggest mistake of our lives.

## Why did we do this? <sup> Because Jafari made us </sup>

Traditional VLA Robots are extremely expensive, requiring high-end equipment/hardware. This means that they are mostly accessible only to big labs or large companies. This means that they are impractical use for most consumers because of this cost barrier. We built a pipeline that implements a VLA model on a low-cost collaborative robot (MyCobot 280) that can be extended to robots that have similar capabilites (Python API). It is affordable bringing VLA capabilities to a low-cost robot instead of expensive lab hardware, accessible, enabling students, small teams, and hobbyists to experiment with VLA models, and practical, demonstrating that VLA concepts can work in everyday, low-budget settings.

![Not 20,000 Dollars.](https://m.media-amazon.com/images/I/51iyDskD-2L.jpg)

## Design Methodology
We used the Software Developement Life Cycle (SDLC) V-Model, a variation of the V-Model introduced in class.

![SDLC V-Model](https://media.geeksforgeeks.org/wp-content/uploads/20231030123258/software-Testing-SDLC-V-model.webp)

### V-Model Verification Phases
1. Requirement Analysis <kbd>&rarr;</kbd> The cobot is able to understand and respond to our queries.
2. System Design <kbd>&rarr;</kbd> Large Chunks: Creating Datasets, Training Model, Running Cobot.
3. Architectural Design <kbd>&rarr;</kbd> Using DROID Dataset, $\pi_0$, pymycobot.
4. Module Design <kbd>&rarr;</kbd> See objects inside the graph.
5. Coding Phase <kbd>&rarr;</kbd> Actually coding.

### V-Model Validation Phases
1. Unit Testing <kbd>&rarr;</kbd> Figuring out bugs in the code.
2. Integration Testing <kbd>&rarr;</kbd> Figuring out if our modules actually communicate with each other properly (e.g. joint velocities)
3. System Testing <kbd>&rarr;</kbd> See if the cobot works.
4. User Acceptance Testing <kbd>&rarr;</kbd> See if our cobot works for clients.

## Performance Metrics
For safety, we looked to the only standard mentioning cobots, [ISO/TS 15066:2016](https://www.iso.org/standard/62996.html), which describes safety standards for cobots in manufacturing environments. While we did take design cues from this guideline, such as having an easily accessible stop function (unplugging the cobot), as our cobot is autonomous, not all requirements are feasible, such as the hand-guided requirement. Fortunately, our cobot is much too small and slow to do any real harm, and its reach is limited.

We committed to following the NSPE Code of Ethics as a practical checklist for how we built and deployed the system. First, we **held paramount public safety, health, and welfare** by designing for safe behavior by default: conservative speed/torque limits, clear work zones, and an immediate, reliable emergency stop/kill path; We also staged autonomy carefully: bench tests, then constrained trials, then supervised operation, before we ever let the model act near people or valuable equipment. We **worked within our competence** by treating the VLA model like a safety-critical component: We documented assumptions, uncertainty, and known failure modes; We avoided claiming capability beyond what we have validated; and when we hit domains that require deeper expertise (mechanical safety, controls, ML safety, cybersecurity), we sought review, follow standards/guidance, and redesign. We **communicated truthfully** by reporting results and limitations without overstatement: clear metrics, reproducible evaluations, and explicit notes about edge cases, data gaps, and when the model is likely to be wrong, so no one mistakes a demo for a production-ready system. We **acted as faithful trustees** for anyone impacted by the project by protecting camera feeds and recorded logs: We minimize what we collected, get consent when people might be captured, secure storage and transfers, and treat partner/lab information as confidential. Finally, we **conduct myself honorably and responsibly** by respecting licenses and attribution for frameworks/datasets, building basic cybersecurity into the networked control loop (safe defaults, authentication where appropriate, and limiting exposure), and continuously improving the system so it’s not only impressive, but demonstrably safe, transparent, and accountable.

For performance, we based our results off of the average task progress from the [pi_0 report](https://arxiv.org/html/2410.24164v1). As VLA models are black-box models, there is unfortunately no way to set any concrete standards for our scenario. However, knowing that there is no support for our cobot (there is no model made for the MyCobot 280, nor does the MyCobot 280 even support LeRobot), to be quite candid, having it work at all was our main goal, especially with a more lightweight model and limited training data. In some scenarios, even Physical Intelligence was unable to make any progress with third-party robots using their "small" model, so any movement in our cobot is very big deal.

## Science!
Why this was the largest mistake of our lives.

A Vision–Language–Action (VLA) model is, mathematically, a **conditional policy** that we can write as
$$\pi_\theta(a_t \mid o_t, \ell),$$
where (o_t) is the robot’s observation at time (t) (multi-camera images plus proprioception like joint angles/gripper state), (\ell) is a language instruction, and (a_t) is the action (e.g., joint deltas/velocities or end-effector targets). The “science” is that we’re learning a high-dimensional function that fuses vision and language into a control-relevant latent representation, then maps that representation into the robot’s action space.

DROID-style dataset collection is basically building an empirical approximation to the trajectory distribution we want the policy to imitate. We log episodes as time-indexed trajectories
$$\tau = {(o_t,\ell,a_t)}_{t=1}^{T},$$
with **synchronized** multi-camera images, proprioception, and actions, plus timestamps and metadata. The “science” here is data quality: alignment (camera frames matched to the exact action/state), consistent coordinate frames, calibrated scaling of actions, and coverage of the state space (diverse starts, perturbations, recoveries). Good datasets reduce covariate shift and make the supervised objective meaningful; bad synchronization or inconsistent action definitions can dominate the loss and prevent the model from learning a usable policy.

Training is usually **imitation learning / behavior cloning**, i.e., supervised learning on demonstrations (\mathcal{D}={(o_t,\ell,a_t)}). We optimize parameters (\theta) to maximize the likelihood of expert actions (or equivalently minimize a loss), commonly:
$$\min_\theta \ \mathbb{E}*{(o_t,\ell,a_t)\sim\mathcal{D}}\big[-\log \pi*\theta(a_t \mid o_t,\ell)\big]$$
for discrete actions, or a regression loss like MSE / negative log-likelihood under a Gaussian for continuous actions:
$$\min_\theta \ \mathbb{E}\big[|a_t-\mu_\theta(o_t,\ell)|^2\big] \quad \text{or} \quad \min_\theta \ \mathbb{E}\big[-\log \mathcal{N}(a_t;\mu_\theta,\Sigma_\theta)\big]$$
Under the hood, the model is doing representation learning: a vision encoder turns pixels into features, a language encoder turns text into embeddings, and a fusion module (often attention) computes a context vector that an action head maps into controls. The practical math issues we deal with are distribution shift (train on demonstrations, test on the model’s own states), stability (small errors can compound over time), and choosing an action parameterization that makes the learning problem well-conditioned (scaling/normalizing actions, predicting deltas vs absolute targets, discretizing gripper, etc.).

LoRA fine-tuning is an efficiency trick grounded in linear algebra. Instead of updating a massive pretrained weight matrix (W\in\mathbb{R}^{d\times k}), we freeze it and learn a low-rank update:
$$W' = W + \Delta W,\qquad \Delta W = BA,$$
where (B\in\mathbb{R}^{d\times r}), (A\in\mathbb{R}^{r\times k}), and (r \ll \min(d,k)). This constrains the adaptation to a low-dimensional subspace, drastically reducing trainable parameters and making optimization cheaper while still allowing the model to specialize to our robot/cameras. In attention layers, this is especially effective because the key/query/value projections are large, and low-rank updates can capture a lot of domain shift (new camera geometry, lighting, robot kinematics) without retraining everything.

Inference is the real-time application of the learned policy: at each step we compute
$$a_t = f_\theta(o_t,\ell)$$
(or sample (a_t \sim \pi_\theta(\cdot \mid o_t,\ell))), then execute it in a control loop. The systems side matters: we pick a policy rate (say 10–30 Hz), synchronize sensors, and apply filtering/constraints so actions are physically feasible:
$$a_t^{\text{safe}} = \text{clip}(\text{filter}(a_t),\ \text{joint limits, velocity limits, workspace limits}).$$
Even if the policy is learned, the execution still leans on classical control ideas—rate limiting, smoothing, and constraint projection—because the robot is a dynamical system and we need stability and safety margins.

