# Cobotic AI
The biggest mistake of our lives.

## Why did we do this? <sup> Because Jafari made us </sup>

Traditional VLA Robots are extremely expensive, requiring high-end equipment/hardware. This means that they are mostly accessible only to big labs or large companies. This means that they are impractical use for most consumers because of this cost barrier. We built a pipeline that implements a VLA model on a low-cost collaborative robot (MyCobot 280) that can be extended to robots that have similar capabilites (Python API). It is affordable bringing VLA capabilities to a low-cost robot instead of expensive lab hardware, accessible, enabling students, small teams, and hobbyists to experiment with VLA models, and practical, demonstrating that VLA concepts can work in everyday, low-budget settings.

![Not 20,000 Dollars.]([https://www.1x.tech/_next/image?url=https%3A%2F%2Fcdn.sanity.io%2Fimages%2Fqka6yvsc%2Fproduction%2Fba6083f5add3b381b4034a9176609e92e75e29ec-1100x655.jpg%3Fw%3D2440%26q%3D100%26fit%3Dmax%26auto%3Dformat&w=3840&q=75](https://shop.elephantrobotics.com/cdn/shop/collections/cobot_collaborative-robot_6dof-robitc-arm_6-axis-robot-arm-for-education_ROS-robot_6-DoF-collaborative-robot-M5stack-1.jpg?v=1689736736))

## Design Methodology
We used the SDLC V-Model, a variation of the V-Model introduced in class.

### V-Model Verification Phases
1. Requirement Analysis -> The cobot is able to understand and respond to our queries.
2. System Design -> Large Chunks: Creating Datasets, Training Model, Running Cobot.
3. Architectural Design -> Using DROID Dataset, $\pi_0$, pymycobot.
4. Module Design -> See objects inside the graph.
5. Coding Phase -> Actually coding.

### V-Model Validation Phases
1. Unit Testing -> Figuring out bugs in the code.
2. Integration Testing -> Figuring out if our modules actually communicate with each other properly (e.g. joint velocities)
3. System Testing -> See if the cobot works.
4. User Acceptance Testing -> See if our cobot works for clients (we haven't completed this step).

## Performance Metrics
For safety, we looked to the only standard mentioning cobots, [ISO/TS 15066:2016](https://www.iso.org/standard/62996.html), which describes safety standards for cobots in manufacturing environments. While we did take design cues from this guideline, such as having an easily accessible stop function (unplugging the cobot), as our cobot is autonomous, not all requirements are feasible, such as the hand-guided requirement. Fortunately, our cobot is much too small and slow to do any real harm, and its reach is limited.

For performance, we based our results off of the average task progress from the [$\pi_0$ report](https://arxiv.org/html/2410.24164v1). As VLA models are black-box models, there is unfortunately no way to set any concrete standards for our scenario. However, knowing that there is no support for our cobot (there is no model made for the MyCobot 280, nor does the MyCobot 280 even support LeRobot), to be quite candid, having it work at all was our main goal, especially with a more lightweight model and limited training data. In some scenarios, even Physical Intelligence was unable to make any progress with third-party robots using their "small" model, so any movement in our cobot is very big deal.
