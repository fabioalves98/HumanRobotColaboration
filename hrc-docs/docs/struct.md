# Structure

## 1 - Introduction

> Introdução do capítulo? E nos próximos capítulos?

#### Motivation

> Mini explicação da necessidade de colaboração humano robot. Especificar que este caso é especifico a manipuladores robóticos
>
> - Incorporar Industria 5.0 e diferenças para a 4.0 
>   - https://www.mastercontrol.com/gxp-lifeline/3-things-you-need-to-know-about-industry-5.0/
>   - https://ec.europa.eu/info/publications/industry-50_en

> O porque deste tema? O origem / necessidade de criar tarefas colaborativas entre robot e humano. Dar exemplos reais de colaboração (industria, produção, tarefas). Dar enfase na presença do humano no ambiente e em tarefas dinamicas que não sejam hardcoded

> Vantagens de criar métodos que facilitem a criação e execução de tarefas colaborativas entre robot e humano. Explicar que as frameworks existentes são especificas para cada robot / marca e que há necesisdade de criar um ambiente genérico e "platform agnostic"

> Incluir o papel do IRIS no trabalho?

> Explicar sucintamente o que esta dissertação irá resolver / contribuir para o tema (explicar que é mais uma dissertação sobre cirar ferramentas para a criação de tarefas humano robot e não apenas sobre as tarefas em si)

#### Objectives

> Explicar especificamente os objetivos desta dissertação, tendo em conta a proposta e o trabalho que foi realizado. Explicar que mais do que criar tarefas, foram criadas ferramentas que promovem a colaboração humano robot

> Listar objetivos? Provavelmente sim...

> Após listar os objetivos (ferramentas), listar as tarefas que foram alcançadas com essas ferramentas

> Falar também sobre rqt_sami e rqt_cobot

> Se for feito, explicar inclusão dos trabalhos desta dissetação com iris_cork

> Se for feito, explicar que seram ralizados testes com pessoas e com o robot

#### Outline

> Explicar o que vai ser dito em cada capítulo. Fazer uma descrição mais detalhada, um paragrafo para cada capitulo



## 2 - Collaborative Robotics

> Introdução do capítulo?

#### Collaborative Robotics

>   O que é? Como? Onde?
>
>   - Definir a área
>   - Definir o que é um robot colaborativo e para que são utilizados
>     - Dar exemplos utilizando notícias, estatisticas e papers do paper de review
>   - Definir ambientes colaborativos e a sua percepção

> Subcategorizar as várias componentes de colaboração humano robot
>
> - Hardware
> - Segurança
> - Programação e controlo de robots
> - Interação Humano Robot

> Possivelmente referir teses antigas feitas no IRIS que utilizam braços robóticos

> Explicar que esta tese foca-se em real time colaborative robotics e não apenas em planeamento estático com MoveIt

>   Conhecimento teórico necessário para abordar o tema
>
>   - Transformations
>   - Path Planning
>   - Robotic manipulators control (500Hz)
>   - FK, IK, Jacobian conversion
>   - Controladores, se forem utilizados
>   - (Ir buscar este conteúdo ao livro da Springer sobre Robótica)

#### Predominant Technologies

>   Tecnologias (software) frequentemente utilizado
>
>   Falar que as plataformas de controlos dos robots são diferentes para cada robot e geralmete closed source
>
>   Talvez falar sobre o UR Teach Pendant e modos de controlo do UR10e (paper)
>
>   Comparar os modos de controlo (URScript, ur_rtde, ROS Driver)
>
>   ROS - Senso uma das tecnologias principais, dar alguma enfase e explicar a escolha de ROS 1 em vez de ROS 2 
>
>   UR_Driver, MoveIt, ur_rtde, RViz, dynamic_reconfigure, rqt_plugins, ros_smach, plotjuggler
>
>   PCL, Euclidean Clustering, RANSAC

#### Existing Approaches

>Trabalhos realizados no tema - Pesquisar mais
>
>KUKA Sunrise Toolbox

#### Discussion ?

> Tendo em conta o que foi dito anteriormente retirar conclusoes que justifiquem o trabalho que foi feito
>
> Mais uma vez, dar enfase nas restrições que existem em software proprietario e na falta de algoritmos e metodologias abrangentes e open source para a criação de tarefas colaborativas
>
> Usar o UR teach pendant como exemplo, apontar falhas e explicar como o que se segue as pode resolver
>
> Bom exemplo de como fazer um capítulo destes na tese dos drones



## 3 - Collaborative Setup

#### Robotic Manipulator

#### Vision and Sensors

#### Shared Workspace and Tools




## 4 - Hand Guiding

#### Force / Torque Sensor Correction

#### End Effector Weight Compensation

#### Force / Torque to Robot Motion



## 5 - Safe Path Planing

#### Obstacle Detection

#### Potential Fields Method

##### Attraction

##### Repulsion

##### Controller



## 6 - Collaborative Tasks

> Método de definir tarefas industriais (não colaborativa) baseadas me primitivas (waypoints)

#### Tool Transfer

#### Object Lifting Assistant

#### ...



## 7 - Experiments and Results



## 8 - Conclusion





## Material Developed

-   Interaction with robot through gripper taps in multiple directions
-   Time series analysis of FT to detect interaction with the robot

-   rqt_sami - GUI plugin to control de robot from a computer based in iris_sami functionalities
-   rqt_cobot - GUI plugin to interact with the robot based on the colaborative tasks of iris_cobot
-   Visual feedback through gripper LEDs
-   Plotjuggler integration
-   ros_smach global state machine



## Extra

- Links de notícias / páginas em footnote, e não nas referências
- Nomenclature

