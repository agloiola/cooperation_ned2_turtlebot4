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
)(Ubuntu 24.04)
- Python 3
- RViz2
- [MoveIt2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

## Instalação

Siga os passos abaixo para configurar o ambiente e instalar todas as dependências necessárias:

1. Clone este repositório dentro do seu workspace ROS2. Ele contém os algoritmos e arquivos necessários para a cooperação entre o TurtleBot4 e o NED2 durante a tarefa de pick and place:

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
