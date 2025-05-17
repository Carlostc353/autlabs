#!/usr/bin/env python3
#   Este nodo se suscribe a una imagen de ROS, la convierte en una matriz de
#   OpenCV y la muestra en pantalla
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

majinBooClassif = cv2.CascadeClassifier('cascade.xml')

class Cam(object):
  def __init__(self, topic_name="camera_frame"):
    self.bridge = CvBridge()
    self.image = np.zeros((10,10))
    isub = rospy.Subscriber(topic_name, Image, self.image_callback)

  def image_callback(self, img):
    self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")

  def get_image(self):
    return self.image


if __name__ == '__main__':

  # Inicializar el nodo de ROS
  rospy.init_node('camera_node')

  # Objeto que se suscribe al tópico de la cámara
  topic_name = "/camera/rgb/image_raw"
  cam = Cam(topic_name)

  # Tópico para publicar una imagen de salida
  topic_pub = 'image_out'
  pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)

  # Frecuencia del bucle principal
  freq = 10
  rate = rospy.Rate(freq)
  # Bucle principal
  while not rospy.is_shutdown():
    
    # Obtener la imagen del tópico de ROS en formato de OpenCV
    I = cam.get_image()
    frame = I
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    toy = majinBooClassif.detectMultiScale(
        gray,
        scaleFactor=1.3,
        minNeighbors=10
    )

    for (x, y, w, h) in toy:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, 'Majin Boo', (x, y - 10), 2, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    
    # Realizar algún tipo de procesamiento sobre la imagen
    
    
      
    # Mostrar la imagen
    cv2.imshow("Imagen Camara Turtlebot3", I)

    # Esperar al bucle para actualizar
    cv2.waitKey(1)
    # Opcional: publicar la imagen de salida como tópico de ROS
    #pubimg.publish(cam.bridge.cv2_to_imgmsg(I))
    rate.sleep()

cv2.destroyAllWindows()

