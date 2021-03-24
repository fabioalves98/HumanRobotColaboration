# Braço Robótico e Humano Desempenham Tarefas de forma Colaborativa

# State of the Art

## 1 - 2019 - Human–robot interaction in industrial collaborative robotics: a literature review of the decade

- **HRI -** the process of conveying human intentions and interpreting task descriptions into a sequence of robot motions complying with robot capabilities and working requirements

#### Criteria for HRI

- **Workspace:** the overlapping space in the working range of human and robot is described as the common workspace.
- **Working time:** it is defined as the time the participant is working inside the workspace.
- **Aim:** every entity of the interacting team has an aim to achieve. This aim can match or mismatch with the other one.
- **Contact:** since human and robot share the same workspace, they may come into contact with each other either [16] (i) occasionally or by accident if normal operation is intended to be without contact, or (ii) on purpose if the operator is supposed to work in contact with the robot, exchanging forces and cooperating in action upon on the environment.

#### Classification of HRI

- **Human–Robot Coexistence (HRCx)**, also called coaction, is defined [17] as the capability of sharing the dynamic workspace between humans and robots without a common task (operate on dissimilar tasks) [18,19] or, without requiring mutual contact or coordination of actions [20] and intentions (human and robot may have different aims) [21]. It is generally limited to collisions avoidance.
- **Human–Robot Cooperation (HRCp)** acts on a higher level [22] than HRCx. In such a case, humans and robots are working on the same purpose and fulfill the requirements of time and space, simultaneously. The cooperation requires thus advanced technologies such as force-feedback sensing or advanced machine vision [1,17], and far more sensing techniques for collision detection and avoidance.
- **Human–Robot Collaboration (HRC)** is the feature of performing a complex task with direct human inter-action in two different modalities [21]: 
  - (i) Physical collaboration where an explicit and intentional contact with forces exchange exists between human and robot [23]. By measuring or estimating these forces/torques [10], the robot can predict human motion intentions and react accordingly [24,25]. 
  - (ii) Contactless collaboration where no physical interaction exists. In such a case, actions are coordinated from information exchange which can be achieved via direct communication (speech, gestures, etc.), or indirect communication (intentions recognition, eye gaze direction, facial expressions, etc.) [26,27]. In such scenarios, the operator performs task parts requiring dexterity or decision-making, while the robot realizes parts that are not well suited to direct human involvement (repetitive or high-force applications, chemical deposition, precision placement, etc.)

<img src="screenshots/papers/1/1.png" width=100%>

<img src="screenshots/papers/1/2.png" width=50%>

<img src="screenshots/papers/1/3.png" width=100%>

> Boa tabela de comparação de artigos

#### Safety in Industrial Robots

<img src="screenshots/papers/1/4.png" width=100%>

- Distributed real-time approach based on a 3D simulation [117]
  - 121, 122, 123, 304
- Real-time collision avoidance approach based on depth sensor [119]
- Pre-collision algorithms and virtual reality tools [154]

<img src="screenshots/papers/1/5.png" width=60%>

> Capítulo com bué conteudo em referências

#### Cognitive Human Robot Interactions

<img src="screenshots/papers/1/6.png" width=90%>

- Human actions recognition [192-194]
- Gestures recognition [201]
  - Control interface to teleoperate robot based on hand gestures using ROS [304]
- Faces Recognition [202]
- Voice Commanding [206]
- Social gaze and social acceptance [195]

#### Robot Programming Approaches

<img src="screenshots/papers/1/7.png" width=90%>

- Generation of Robotic Skills [216]
- Augmented and Virtual Reality [215]
- On-line Programming [217]
- Programming by Demonstration
  - Muitas referências sobre ensinar robots a fazer tarefas apenas por demonstração

#### Human Robot Tasks Allocation

<img src="screenshots/papers/1/8.png" width=90%>

- Ontology-based Knowledge
  - Simplify user interface [93, 222]
  - Generic knowledge based system architecture for cobots [40]
- Creating high-level tasks plans
  - Behavior Trees [228]
  - CoSTAR framework [229]
  - Skill Based System software in ROS [304]
  - XRob platfoem for HRI [230]
  - Architecture for interactive multi-modal industrial HRI [231]
- Tasks Allocation and Scheduling
  - Decision-making method that allows human-robot task allocation integrated within ROS [197]
  - Allocating tasks to humans and robots for cell manufactoring [235]
  - Analytic Hirarchy Process as a decision-making approach and Hierarchical Task Analysis [324]

#### Fault Tolerance

- Error Detection
- Error Diagnosis
- Recovery

### Papers Para Ler

- [7] - Human centered assistance applications for the working environment of the future. 2015
- [14] - Evaluation of flexible graphical user interface for intuitive human robot interactions. 2014
- [18] - Integration of active and passive compliance control for safe human-robot coexistence. 2009
- [19] - A design approach for incorporating task coordination for human-robot coexistence within assembly systems. 2015
- [21] - Integrated control for PHRI: collision avoidance, detection, reaction and collaboration. 2012
- [22] - A brief review on safety strategies of physical human-robot interaction. 2019
- [23] - A depth space approach for evaluating distance to objects. 2015
- [24] - Multimodal control for human-robot cooperation. 2013
- [27] - Planning safe and legible hand-over motions for human-robot interaction. 2010
- [28] - Study on application of a human-robot collaborative system using hand-guiding in a production line. 2016
- [44] - Ros based coordination of human robot cooperative assembly tasks. 2015
- [49] - Human-robot physical interaction and collaboration using an industrial robot with a closed control architecture. 2013
- [55] - Optimized assistive human–robot interaction using reinforcement learning. 2016
- [60] - Introducing robots without creating fear of unemployment and high cost in industries. 2018
- [70] - Human-robot collaboration for tooling path guidance. 2016
- [88] - Real-time computation of distance to dynamic obstacles with multiple depth sensors. 2016
- [100] - Working together: a review on safe human-robot collaboration in industrial environments. 2017

- [325] - Key challenges and open issues of industrial collaborative robotics. 2018

## 2 - 2019 - Human–Robot Collaboration in Manufacturing Applications: A Review

- Concept of cobots invented in 1996 [2]
- Classification of human robot interaction [7]
  - **Coexistence - **Human operator and cobot are in the ame environment but do not interact with each other
  - **Synchronised - **Human operator and cobot work in the same workspace, but at different times
  - **Cooperation - **Human operator and cobot work in the same workspace at the same time, though focusing on separate tasks
  - **Collaboration - **Human operator and the cobot must execute a task together, the action of the one has immediate consequences on the other, thaks to special sensors and vision systems

<img src="screenshots/papers/2/1.png" width=90%>

- Other classifications [8-11]
- Safety requirements for cobots
  - **Safety-rated Monitored Stop (SMS) - **used to cease robot motion in the collabortice workspace before an operator enter the collaborative workspace
  - **Hand-guiding (HG) - **where an operator uses a hand-operated device, locate at or nead the robot end-effector, to transmit motion commands to the robot
  - **Speed and separation monitoring (SSM) - **where the robot system and operator may move concurrently in the collaborative workspace. During robot motion, the orbot system never gets closer to the operator than the protective separaton distance
  - **Power and force limitting (PFL) - **where the robot system shall be designed to adequatly reduce risk to an operator by not exceeding the applicable threshlod limit values
- Defined in [13], cobots should be equipped with additional features such as force and toque sensors, force limits, vision systems (cameras), laser systems, anti-collision systems, recognition of voice commands, and / or system to coordinate the actions of human operators with their motion
- Robot learning from demonstratoin [15] - **IMPORTANTE**
- Table comparing humans, cobots and tobots
  - Assembly - attatching 2 or more components
  - Placement - positiong each part in the proper position
  - Handling - manipulation of the picked part
  - Picking - tacking from the feeding point

<img src="screenshots/papers/2/2.png" width=90%>

- Collaborative robots are especially advantageous for assembly tasks, particularly if the task is executed with a human operator. They are also suitable for pick and place applications, though the adoption of a traditional robot or a handling system can offer better results in terms of speed, precision, and payload
- Aplications of cobots in this review
  - Assembly - when the cobot collaborated with the operator in an assembly process
  - Human Assistance - when the cobot acts as an ergonomic support for the operator
  - Machine Tending - when the cobot performs loading / unloading operations
- Physical human-robot interaction in 6DOF [28] - **IMPORTANTE**
- End-effector precise hand-guiding for collaborative robots [30] - **IMPORTANTE**
- **Results of the review**
  - Cobots are being researched more than tobots
  - The most used control system is vision
  - The most used methodologie was hand guiding but the others were aso used
  - The most researched task was assembly, by a large margin
  - To increase safety, productivity and task performance, researchers will need to improve planners, environment and task understanding, operator intention understanding and ergonomic cell setups
  - To imporve HRI systems, common future work focuses on increasing the robots' and operators' awareness of the task and environment by object redognition and integrating multi-modal sensing in an intuitive manner for the operator
- **Trends of Market**
  - Robot market is going ot grow
  - The fall in robot prices has led to a growing market for cobots
  - Small and medium sized enterprises could not afford robotic applications due to the high capital costs
- Trust-based compliant robot-human handovers of payloads [29] - **IMPORTANTE**
- Table with the reviewed papers