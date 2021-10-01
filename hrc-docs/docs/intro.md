# Introduction

> Introdução do capítulo? E nos próximos capítulos?

### Motivation

- An industrial robot is a machine that allows efficiency, strength and automation on the production line. They can be seen as substitutes for humans for their ability to perform repetitive and tedious manufacturing tasks autonomously and with high accuracy and precision. By default, and for safety reasons they are intended to work separately from humans, in their own environment and are usually programmed to stop if a human worker enters its space.
- Even with this great abilities, there are some tasks that are either too complex, or too dynamic to fully automate and require the continuous presence of a human worker, to either supervise or assist the industrial robot. This need of shared execution of tasks with robots, resulted in the emergence of Collaborative Robots, also known as Cobots. These are industrial robots which in general, are built with lightweight materials, equiped with force/precision assistance hardware and speed limitations, are easy to set up and program which altogether, promotes safe and efficient interaction  with humans.
- Starting with a cobot, to reach the collaborative execution of a task, there is a gap which this Dissertation aims to fill, by proposing a set of tools, techniques and workflows that not only leverage on the hardware of cobots, but also use external sensors to promote a shared environment where humans and cobots can safely execute tasks together.



> Mini explicação da necessidade de colaboração humano robot. Especificar que este caso é especifico a manipuladores robóticos

> - Incorporar Industria 5.0 e diferenças para a 4.0 
>   - https://www.mastercontrol.com/gxp-lifeline/3-things-you-need-to-know-about-industry-5.0/
>   - https://ec.europa.eu/info/publications/industry-50_en

> O porque deste tema? O origem / necessidade de criar tarefas colaborativas entre robot e humano. Dar exemplos reais de colaboração (industria, produção, tarefas). Dar enfase na presença do humano no ambiente e em tarefas dinamicas que não sejam hardcoded

> Vantagens de criar métodos que facilitem a criação e execução de tarefas colaborativas entre robot e humano. Explicar que as frameworks existentes são especificas para cada robot / marca e que há necesisdade de criar um ambiente genérico e "platform agnostic"

> Incluir o papel do IRIS no trabalho?

> Explicar sucintamente o que esta dissertação irá resolver / contribuir para o tema (explicar que é mais uma dissertação sobre cirar ferramentas para a criação de tarefas humano robot e não apenas sobre as tarefas em si)

## Objectives

- This Dissertation has as it's main objective the promotion of shared execution of tasks, between humans and industrial robots, specifically targetting collaborative robotic manipulators. To do so, it aims to develop the following set of techniques:
  - Interaction and communication with robot through touch, by executing taps and double taps on the End Effector (EEF)
  - Manipulation of the robot by physically hand guiding it
  - Dynamic compensation of weight coupled to the EEF, such as tools, grippers and objects
  - Detection of dynamic obstacles in the environment
  - Safe motion planning aware of dynamic obstacles
  - Creation of high-level task plans

- With theses techniques, this Dissertation also aims to achieve the following collaborative tasks:
  - Transfer of tools and objects between human and robot
  - Lift assistance and precise manipulation of heavy objects
  - Collision free execution of an industrial task
- Besides the main objectives, efforts were made to increase the ease of use of a robotic manipulator, with the development of 2 graphical toolbox interfaces, which help control and monitor the status of the cobot and the task being executed. Finally, all techniques were deployed in a modular task-level architecture, which allows the user to rapidly incorporate new collaborative tasks and easily monitor their execution.



> Explicar especificamente os objetivos desta dissertação, tendo em conta a proposta e o trabalho que foi realizado. Explicar que mais do que criar tarefas, foram criadas ferramentas que promovem a colaboração humano robot

> Listar objetivos? Provavelmente sim...

> Após listar os objetivos (ferramentas), listar as tarefas que foram alcançadas com essas ferramentas

> Falar também sobre rqt_sami e rqt_cobot

> Se for feito, explicar inclusão dos trabalhos desta dissetação com iris_cork

> Se for feito, explicar que seram ralizados testes com pessoas e com o robot

## Outline

>  Explicar o que vai ser dito em cada capítulo. Fazer uma descrição mais detalhada, um paragrafo para cada capitulo

- This Dissertation is divided in 7 chapters
- Chapter 2 (Collaborative Robotics) starts by giving an introductory context on the field of Human-Robot Collaboration, its relevance, achievements and problems. Then, it gives an analysis on existing approaches and technologies both commercial and open-source. It concludes by discussing the framing of this Dissertation inside this field, what it aims to solve and contribute.
- Chapter 3 (Hand Guided Object Manipulation) explains the techniques developed that are cobot-centered. It shows how the forces and torques felt at the EEF of the robot are used to achieve hand guidance with weight compensation.
- Chapter 4 (Dynamic Obstacle Avoidance) explains the techniques developed that are environment-centered, such as the use of an external depth camera to detect obstacles, and the implementation of a online collision avoidance algorithm.
- Chapter 5 (Collaborative Tasks) showcases the use of the previously developed techniques to execute a set of tasks between a human and a robot.
- Chapter 6 (Experiments and Results) presents the experiments made to validate the techniques and the respective results.
- Chapter 7 (Conclusion) gives the final regards about the developed work and elaborates on future work.



****

#### Recursos

- 2019 - Human–robot interaction in industrial collaborative robotics: a literature review of the decade
- 2019 - Human–Robot Collaboration in Manufacturing Applications: A Review

