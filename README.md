# Cooperação entre NED2 e TurtleBot4 para tarefas de Pick and Place

Este projeto tem como objetivo desenvolver uma aplicação real que demonstre a cooperação entre o robô móvel TurtleBot4 e o braço robótico Niryo NED2 para a execução de tarefas de manipulação de objetos.

O sistema integra navegação móvel, percepção visual e manipulação robótica. O TurtleBot4 executa um algoritmo de reposicionamento para alinhar o objeto na melhor orientação em relação ao manipulador robótico, permitindo que o braço NED2 realize a tarefa de pick and place de forma precisa e eficiente.

A solução utiliza ROS2 Jazzy, MoveIt2 e algoritmos de navegação e reposicionamento para coordenar os dois sistemas robóticos.

---

## Índice

- [Arquitetura do Sistema](#arquitetura-do-sistema)
- [Pré-requisitos](#pré-requisitos)
- [Instalação](#instalação)
- [Execução](#execução)

## Arquitetura do Sistema

O sistema é baseado na comunicação entre diferentes nós ROS2, responsáveis por navegação, percepção e manipulação.

O fluxo de execução ocorre da seguinte forma:

1. O TurtleBot4 navega até a área de manipulação transportando o objeto a ser apreendido.
2. A câmera detecta a posição do objeto no ambiente.
3. Um algoritmo de reposicionamento ajusta a orientação do robô móvel, em parceria com Planejamento de movimento do braço robótico utilizando MoveIt2.
4. O manipulador NED2 executa a tarefa de pick and place;

Essa arquitetura permite que o robô móvel e o manipulador trabalhem de forma cooperativa, aumentando a flexibilidade da manipulação no ambiente.

## Pré-requisitos

Antes de começar, certifique-se de que você tenha os seguintes softwares instalados e configurados:

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html
) (Ubuntu 24.04)
- Python 3
- RViz2
- [MoveIt2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

## Instalação

Siga os passos abaixo para configurar o ambiente e instalar todas as dependências necessárias:

1. Clone este repositório dentro do seu workspace ROS2. 

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/agloiola/cooperation_ned2_turtlebot4.git
    ```

2. Instale o pacote do driver ROS2 do NED2, responsável pelo controle do braço robótico:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/NiryoRobotics/ned-ros2-driver.git
    ```

3. Instale os pacotes do TurtleBot4:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/turtlebot/turtlebot4.git
    ```

4. Instale o pacote usb_cam, utilizado para acessar a câmera conectada ao sistema:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/ros-drivers/usb_cam.git
    ```

5. Para a detecção da posição do objeto, instale o pacote de marcadores AprilTag:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/christianrauch/apriltag_ros.git
    ```

6. Após instalar todos os pacotes, compile o workspace e carregue o ambiente do ROS2:

    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
## Execução

Antes de iniciar a execução do sistema, certifique-se de que os robôs estejam ligados, configurados corretamente para comunicação com o computador e com o horário sincronizado com o sistema.

Nos experimentos deste projeto, a conexão foi realizada da seguinte forma:

- o TurtleBot4 foi conectado ao computador por meio do Wi-Fi (hotspot)
- o NED2 foi conectado via rede Ethernet
- o computador também foi conectado à rede Ethernet, garantindo que ambos estivessem na mesma rede

Também é importante verificar se o arquivo de configuração do driver do NED2 está corretamente preenchido com o endereço IP do robô. O arquivo utilizado é:

```bash
~/ros2_ws/src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml
````

1. Inicialização da câmera e detecção da tag

O primeiro passo da execução é iniciar a câmera e o processo de detecção da tag utilizada para identificar a posição do objeto no ambiente:

```bash
ros2 launch cooperation_ned2_turtlebot4 image_camera_tag.launch.py
```

2. Inicialização do driver do NED2

Em um novo terminal, execute o driver do robô:

```bash
ros2 launch niryo_ned_ros2_driver driver.launch.py
```

Em outro terminal, inicie o MoveIt2:

```bash
ros2 launch niryo_ned2_moveit_config ned2_moveit_launch.py
```

3. Inicialização do TurtleBot4

Em um novo terminal, execute:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 daemon stop
ros2 daemon start

ros2 launch cooperation_ned2_turtlebot4 turtlebot4_navigation_full.launch.py
```

Este launch realiza a inicialização da navegação do TurtleBot4, incluindo localização, planejamento de caminho e visualização.

4. Inicialização da Cooperação

Por fim, em um novo terminal, execute o launch de cooperação, responsável pela execução da tarefa de pick and place, leitura da posição do objeto, navegação e reorientação do TurtleBot4:

```bash
ros2 launch cooperation_ned2_turtlebot4 cooperation_ned2_turtlebot4.launch.py
