#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver_node')
        self.image_sub = rospy.Subscriber('/cambot_1/camera/rgb/image_raw', Image, self.image_callback)
        
        self.bridge = CvBridge()
        self.image_count = 0
        self.latest_image = None
        self.save_dir = '/home/ctorres/lab_ws/src/autlabs/autlab2/src'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        rospy.loginfo("Presiona 's' para guardar una imagen de CAMBOT 1. Ctrl+C para salir.")
        self.loop()

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error al convertir imagen: {e}")

    def loop(self):
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                cv2.imshow("Cambot 1 - Imagen en vivo", self.latest_image)
                key = cv2.waitKey(10)

                if key == ord('s'):
                    filename = os.path.join(self.save_dir, f"cambot1_{self.image_count:04d}.jpg")
                    cv2.imwrite(filename, self.latest_image)
                    rospy.loginfo(f"Imagen guardada en: {filename}")
                    self.image_count += 1

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        ImageSaver()
    except rospy.ROSInterruptException:
        pass

