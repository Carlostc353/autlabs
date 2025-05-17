#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class MultiImageSaver:
    def __init__(self):
        rospy.init_node('multi_image_saver_node')

        self.bridge = CvBridge()
        self.image_count = 0

        self.save_dir = '/home/ctorres/lab_ws/src/autlabs/autlab2/src'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Diccionario para las imágenes más recientes
        self.latest_images = {
            'cambot_1': None,
            'cambot_2': None,
            'cambot_3': None
        }

        # Suscripciones a cada cámara
        rospy.Subscriber('/cambot_1/camera/rgb/image_raw', Image, self.image_callback, callback_args='cambot_1')
        rospy.Subscriber('/cambot_2/camera/rgb/image_raw', Image, self.image_callback, callback_args='cambot_2')
        rospy.Subscriber('/cambot_3/camera/rgb/image_raw', Image, self.image_callback, callback_args='cambot_3')

        rospy.loginfo("Presiona 's' para guardar imágenes de los 3 cambots. Ctrl+C para salir.")
        self.loop()

    def image_callback(self, msg, cambot_name):
        try:
            self.latest_images[cambot_name] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Error al convertir imagen de {}: {}".format(cambot_name, e))

    def loop(self):
        while not rospy.is_shutdown():
            for cambot, img in self.latest_images.items():
                if img is not None:
                    cv2.imshow(cambot, img)
            key = cv2.waitKey(10)

            if key == ord('s'):
                # Guardar imagen de cada cambot si está disponible
                for cambot, img in self.latest_images.items():
                    if img is not None:
                        filename = os.path.join(self.save_dir, "{}_{:04d}.jpg".format(cambot, self.image_count))
                        cv2.imwrite(filename, img)
                        rospy.loginfo("✅ Imagen guardada: {}".format(filename))
                    else:
                        rospy.logwarn("⚠️ No se recibió imagen de {}".format(cambot))
                self.image_count += 1

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        MultiImageSaver()
    except rospy.ROSInterruptException:
        pass
