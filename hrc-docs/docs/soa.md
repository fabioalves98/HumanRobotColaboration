# Collaborative Robotics

-  Introdução do capítulo?

## Background

-   O que é? Como? Onde?
   - Definir a área
   - Definir o que é um robot colaborativo e para que são utilizados
   - Dar exemplos utilizando notícias, estatisticas e papers do paper de review
   - Definir ambientes colaborativos e a sua percepção
- Subcategorizar as várias componentes de colaboração humano robot
  - Hardware
  - Segurança
  - Programação e controlo de robots
  - Interação Humano Robot

- Possivelmente referir teses antigas feitas no IRIS que utilizam braços robóticos

- Explicar que esta tese foca-se em real time colaborative robotics e não apenas em planeamento estático com MoveIt

-   Conhecimento teórico necessário para abordar o tema
   - Transformations
   - Path Planning
   - Robotic manipulators control (500Hz)
   - FK, IK, Jacobian conversion
   - Controladores, se forem utilizados
   - (Ir buscar este conteúdo ao livro da Springer sobre Robótica)

## Predominant Technologies

-   Tecnologias (software) frequentemente utilizado
-   Falar que as plataformas de controlos dos robots são diferentes para cada robot e geralmete closed source
-   Talvez falar sobre o UR Teach Pendant e modos de controlo do UR10e (paper)
-   Comparar os modos de controlo (URScript, ur_rtde, ROS Driver)
-   ROS - Senso uma das tecnologias principais, dar alguma enfase e explicar a escolha de ROS 1 em vez de ROS 2 
-   UR_Driver, MoveIt, ur_rtde, RViz, dynamic_reconfigure, rqt_plugins, ros_smach, plotjuggler
-   PCL, Euclidean Clustering, RANSAC

## Existing Approaches

- Trabalhos realizados no tema - Pesquisar mais
- KUKA Sunrise Toolbox

## Discussion ?

- Tendo em conta o que foi dito anteriormente retirar conclusoes que justifiquem o trabalho que foi feito
- Mais uma vez, dar enfase nas restrições que existem em software proprietario e na falta de algoritmos e metodologias abrangentes e open source para a criação de tarefas colaborativas
- Usar o UR teach pendant como exemplo, apontar falhas e explicar como o que se segue as pode resolver
- Bom exemplo de como fazer um capítulo destes na tese dos drones

