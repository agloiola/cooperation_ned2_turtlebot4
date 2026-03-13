from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Nó do Driver da Câmera (USB Cam)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera',  # Tudo ficará sob /camera/...
            parameters=[{
                'video_device': '/dev/video2',
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'image_width': 640,
                'image_height': 480,
                'camera_name': 'narrow_stereo', # Tem que bater com o YAML
                'camera_info_url': 'file:///home/aline/.ros/camera_info/minha_camera.yaml'
            }]
        ),

        # 2. Nó de Retificação (Image Proc) - Opcional, mas útil
        # Esse nó lê a imagem crua e publica a /camera/image_rect
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            namespace='camera',
            remappings=[
                ('image', 'image_raw'), # O nó espera 'image', ligamos no 'image_raw'
                ('camera_info', 'camera_info')
            ]
        )
    ])