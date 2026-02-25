import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose 
import math
import time

from turtle_interface.srv import ChangeMode
from turtle_interface.action import Punto

class ControladorTortuga(Node):
    def __init__(self):
        super().__init__('controlador_node')
        self.modo_actual = 'manual'
        self.pose_act = None 
        
        self.grupo_callbacks = ReentrantCallbackGroup()

        # Se hereda la función create_publisher y se coloca como tipo de mensaje Twist para alterar el topico /turtle1/cmd_vel
        self.pub_vel = self.create_publisher(Twist, 
                                             '/turtle1/cmd_vel',
                                               10)
        
        # # Para mover la tortuga, nuevamente se hereda una función y de esta forma se modifica el topico, ademas de una función para realizar feedback
        self.sub_pose = self.create_subscription(Pose,
                                                  '/turtle1/pose',
                                                    self.callback_pose,
                                                    10,
                                                    callback_group=self.grupo_callbacks)
        
        # Se usa el servicio para modificar el modo de operación de la tortuga
        self.srv_modo = self.create_service(ChangeMode, 
                                            'cambiar_modo', 
                                            self.callback_cambiar_modo)
        
        # Timer para realizar configuración
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo iniciado. Modo actual: manual')

        # Se declara la acción para realizar la trayectoria de 3 puntos
        self.action_server = ActionServer(
            self, Punto, 'execute_trajectory', 
            execute_callback=self.callback_trayectoria,
            callback_group=self.grupo_callbacks
        )

    def callback_pose(self, msg):
        self.pose_act = msg

    def callback_cambiar_modo(self, request, response):
        self.modo_actual = request.mode
        self.get_logger().info(f'Modo cambiado a: {self.modo_actual}')
        response.success = True
        return response

    # El modo del timer no se activa al usar otros modos que no sean de circunferencia
    def timer_callback(self):
        msg = Twist()
        if self.modo_actual == 'circulo_ah':
            msg.linear.x = 2.0
            msg.angular.z = 1.8 
            self.pub_vel.publish(msg)
        elif self.modo_actual == 'circulo_h':
            msg.linear.x = 2.0
            msg.angular.z = -1.8 
            self.pub_vel.publish(msg)

  
    def mover_a_punto(self, x_obj, y_obj, goal_handle):
        msg = Twist()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Trayectoria cancelada')
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.pub_vel.publish(msg)
                return False

            if self.pose_actual is None:
                time.sleep(0.1)
                continue

            distancia = math.sqrt((x_obj - self.pose_actual.x)**2 + (y_obj - self.pose_actual.y)**2)
            
            if distancia < 0.1: 
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.pub_vel.publish(msg)
                return True 

            angulo_objetivo = math.atan2(y_obj - self.pose_actual.y, x_obj - self.pose_actual.x)
            error_angulo = angulo_objetivo - self.pose_actual.theta
            error_angulo = math.atan2(math.sin(error_angulo), math.cos(error_angulo)) 


            msg.linear.x = 1.0 * distancia
            msg.angular.z = 4.0 * error_angulo

            if msg.linear.x > 1.0: msg.linear.x = 1.0

            self.pub_vel.publish(msg)
            time.sleep(0.1) 

        return False


    def callback_trayectoria(self, goal_handle):
        self.get_logger().info('Iniciando trayectoria :)')
        self.modo_actual = 'trayectoria' 
        

        puntos = [goal_handle.request.pt1, goal_handle.request.pt2, goal_handle.request.pt3]
        resultado = Punto.Result()
        
        for i, punto in enumerate(puntos):
            feedback = Punto.Feedback()
            feedback.current_status = f'Transportandome al punto {i+1} (X:{punto.x}, Y:{punto.y})...'
            goal_handle.publish_feedback(feedback)
            
            exito = self.mover_a_punto(punto.x, punto.y, goal_handle)
            
            if not exito:
                goal_handle.canceled()
                resultado.success = False
                self.modo_actual = 'manual' 
                return resultado

        
        goal_handle.succeed()
        resultado.success = True
        self.modo_actual = 'manual' 
        self.get_logger().info('¡Trayectoria terminada! Regresando a modo manual.')
        return resultado


def main(args=None):
    rclpy.init(args=args)
    nodo = ControladorTortuga()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(nodo, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()