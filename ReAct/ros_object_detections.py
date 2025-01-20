#! /usr/bin/env python3
import rospy
from object_detector_msgs.srv import detectron2_service_server
from sensor_msgs.msg import Image
import time
from sensor_msgs.msg import Image



def detect_objects(rgb):
    rospy.wait_for_service('detect_objects')
    try:
        detect_objects_service = rospy.ServiceProxy('detect_objects', detectron2_service_server)
        response = detect_objects_service(rgb)
        return response.detections.detections
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)



if __name__ == "__main__":

    rospy.init_node("calculate_poses_react")
    try:
        rate = rospy.Rate(10)  # Adjust the rate as needed (Hz)
        while not rospy.is_shutdown():
            # Assuming you have a way to get RGB and depth images
            rgb = rospy.wait_for_message(rospy.get_param('/pose_estimator/color_topic'), Image)
            depth = rospy.wait_for_message(rospy.get_param('/pose_estimator/depth_topic'), Image)

            print('Perform detection with YOLOv8 ...')
            t0 = time.time()
            detections = detect_objects(rgb)
            time_detections = time.time() - t0

            if detections is not None:
                for detection in detections:
                    print(detection.name)
            print()
            print("... received object detection.")


            rate.sleep()

    except rospy.ROSInterruptException:
        pass


