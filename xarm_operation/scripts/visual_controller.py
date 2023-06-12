#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose
from xarm_planner.srv import pose_plan, exec_plan

class detected_object():
    def __init__(self): 
        self.pub = rospy.Publisher("/xarm6_coordinates", Pose, queue_size=10)
        print("Node is running")
        self.data_out = Point()
        self.data_out.x = 0
        self.data_out.y = 0
        self.data_out.z = 0

        # Rango de color del objeto a detectar (en este caso, se utiliza el color azul)
        self.lower_color = np.array([100, 50, 120], dtype=np.uint8)
        self.upper_color = np.array([130, 255, 255], dtype=np.uint8)

        # Inicializa la captura de video desde la cámara
        self.cap = cv2.VideoCapture(2)

        # Cambiar orientación de la webcam
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Obtiene el tamaño de la ventana
        self.window_height = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.window_width = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.center_x = self.window_width // 2
        self.center_y = self.window_height // 2

        # Variable para almacenar el contorno del objeto detectado
        self.object_contour = None

        # Factor de reducción de sensibilidad
        self.sensitivity_factor = 0.1

        # Dimensiones reales del objeto en cm
        self.obj_width_cm = 8
        self.obj_height_cm = 3.5
        self.obj_depth_cm = 2

        # Variables for pid controller
        self.integral = 0
        self.previous_error = 0
        # PID constants
        self.Kp = 0.1
        self.Ki = 0.0
        self.Kd = 0.0
        self.integral = 0
        self.previous_error = 0

    def pid_controller(self, error):
        # Calculate proportional term
        proportional = self.Kp * error
        
        # Calculate integral term
        self.integral = self.integral + (error * self.Ki)
        
        # Calculate derivative term
        derivative = self.Kd * (error - self.previous_error)
        
        # Calculate control output
        control_output = proportional + self.integral + derivative
        
        # Output saturation
        control_output = max(min(control_output, 40), -40)

        # Update previous error
        self.previous_error = error
        
        return control_output

    def position_tracking(self, x, y, z):
        x_gain = 0
        y_gain = 0
        z_gain = 0

        # Controlador para la coordenadas
        if self.y_rel < -1 or self.y_rel > 1:
            x_gain = self.pid_controller(0 - self.y_rel)

        if self.x_rel < -1 or self.x_rel > 1:
            y_gain = self.pid_controller(self.x_rel)

        if self.object_area < 40 or self.object_area > 55:
            z_gain = self.pid_controller(self.object_area-50)

        x += x_gain*0.001
        y += y_gain*0.001
        z += z_gain*0.001

        return x, y, z

    def run(self, xArm_x, xArm_y, xArm_z):
        # Lee un frame de video
        _, frame = self.cap.read()

        # Rotar la imagen 90° a la derecha
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        # Convierte el frame de BGR a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Realiza la segmentación por color dentro del rango especificado
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        # Aplica una operación de apertura para eliminar el ruido
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Encuentra los contornos de los objetos en la máscara
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Encuentra el contorno más grande (objeto dominante)
        if len(contours) > 0:
            object_contour = max(contours, key=cv2.contourArea)

            # Calcula el área del objeto
            self.object_area = int(cv2.contourArea(object_contour) * 0.002)

            print(self.object_area)

            # Obtiene el rectángulo mínimo que rodea al contorno
            rect = cv2.minAreaRect(object_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Dibuja el rectángulo rotado
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

            # Obtiene las coordenadas del rectángulo rotado
            x, y, w, h = cv2.boundingRect(box)

            # Calcula las coordenadas del centro del objeto
            object_center_x = x + (w // 2)
            object_center_y = y + (h // 2)

            # Calcula las coordenadas relativas desde el centro de la imagen
            self.x_rel = round((object_center_x - self.center_x) * self.sensitivity_factor, 3)
            self.y_rel = round((self.center_y - object_center_y) * self.sensitivity_factor, 3)

            # Posicion final del joint TCP
            xArm_x, xArm_y, xArm_z = self.position_tracking(xArm_x, xArm_y, xArm_z)
            print("x: ", xArm_x, "y: ", xArm_y, "z: ", xArm_z)
            
            # Dibuja un punto rojo en las coordenadas del objeto
            cv2.circle(frame, (object_center_x, object_center_y), 5, (0, 0, 255), -1)

        # Dibuja un punto morado en el centro de la imagen
        cv2.circle(frame, (self.center_x, self.center_y), 5, (255, 0, 255), -1)

        # Muestra el frame con las detecciones
        cv2.imshow('Object Detection', frame)

        # Detiene el bucle si se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("Closed from window")

        return xArm_x, xArm_y, xArm_z

    def stop(self):
        # Libera los recursos y cierra las ventanas
        self.cap.release()
        cv2.destroyAllWindows()
        print("Shutting down")

if __name__ == '__main__':
    rospy.init_node("visual_controller")
    
    coordinates = detected_object()
    rate = rospy.Rate(100)
    rospy.on_shutdown(coordinates.stop)
    rate.sleep()

    x = 0.3
    y = 0.0
    z = 0.2

    target_pose = Pose()

    # target_pose.position.x = x
    # target_pose.position.y = y
    # target_pose.position.z = z
    # target_pose.orientation.x = 1.0
    # target_pose.orientation.y = 0.0
    # target_pose.orientation.z = 0.0
    # target_pose.orientation.w = 0.0

    # coordinates.pub.publish(target_pose)

    # start = input("press any key to start traking")

    try:
        while not rospy.is_shutdown():
            x, y, z = coordinates.run(x, y, z)

            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = z
            
            coordinates.pub.publish(target_pose)

            rate.sleep()

    except rospy.ROSInterruptException():
        pass