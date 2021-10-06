# Force Torque Sensor Compensation

## Force / Torque Sensor Correction

##### Apresentar o Sensor

<img src="photos/3-ft_sensor_specs.png" width=30%>

#### Problem

- Valores de força e torque são provenientes de um controlador cujo funcionamento interno é desconhecido e funciona de modo "diferencial"
  - Apenas temos acesso aos valores já processados de Força e Torque
  - Existe uma função de tara do sensor que coloca todos os valores a zero (zero_ft_sensor)
  - O funcionamento interno do controlador adapta-se/varia consoante os valores de Payload e Center of Gravity configurados

- Valores de força e torque apresentam variações em diferentes posições da junta wrist_3 quando, teoricamente, os valores deveiram ser sempre 0

<img src="photos/3-wrist_3_problem_scaled.png" width=100%>

- Drift temporal
  - Se deixarmos o robot parado demasiado tempo os valores de FT iram apresentar variações lineares com o tempo

<img src="photos/3-sensor_drift.png" width=80%>

- Alterações dos valores de força quando são aplicadas forças externas ao sensor
  - Se aplicarmos força no sensor, após o largarmos, os valores de FT vão apresentar variações

<img src="photos/3-sensor_pushes_scaled.png" width=80%>

- Caso especial do eixo Z
  - Valores de força em Z variam consoante a força com que acopolamos um gripper ao EEF
  - Valores de troque em Z variam consoante o movimento do wrist_3

<img src="photos/3-wrist_3_z_problem_scaled.png" width=80%>



#### Proposed Approach

- Gravar os valores de FT em diferentes posições para os corigirmos em tempo real
  - Com recurso a uma função seno/coseno optimizada
  - Com recurso à gravação da médias dos valores obitdos localmente

##### 5 Positions

> Mudar estas fotos para iguais sem o gripper (para não gerar dúvida)

<img src="photos/P1.png" width=20%><img src="photos/P2.png" width=20%><img src="photos/P3.png" width=20%><img src="photos/P4.png" width=20%><img src="photos/P5.png" width=20%>

##### Test in 5 Positions No Gripper

<img src="photos/3-wrist_3_average.png" width=100%>

##### Test in 5 Positions No Gripper | Payload 1.5kg

After correction with previous curve

<img src="photos/3-wrist_3_payload.png" width=70%>

##### Test in 5 Positions No Gripper | Payload 1.5Kg | CoG 10cm Z

After correction with first curve

<img src="photos/3-wrist_3_payload_cog.png" width=70%>

#### Result

- Teste feito na posição P1 sem qualuer peso no EEF

<img src="photos/3-wrist_3_result_scaled.png" width=100%>



## End Effector Weight Compensation

#### Theory Model of FT Sensor

<img src="photos/3-theory_force.jpg" width=50%><img src="photos/3-theory_torque.jpg" width=50%>

#### Result

- Teste onde as duas últimas juntas efetuam rotações e o Robotiq Gripper acopulado

<img src="photos/3-theory_result_2_scaled.png" width=100%>

## Force / Torque to Robot Motion

#### Architecture

<img src="photos/3-architecture.jpg" width=60%>

##### Wrench Filter

<img src="photos/3-wrench_filter.jpg" width=50%><img src="photos/3-wrench_filter_pj.png" width=50%>





