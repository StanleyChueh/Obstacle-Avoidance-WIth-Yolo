import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ultralytics import YOLO
import cv2
import sys
import time

def callback(msg, cmd_vel_pub, rotating, model, cap):
    s1_distance = msg.ranges[0]
    s4_distance = msg.ranges[270]

    # Process camera image
    person_detected = False
    success, img = cap.read()
    
    if success:
        # Object detection with YOLO
        results = model(img, task='detect', mode='predict', imgsz=128)

        # Check if there are any detections
        if results and isinstance(results, list) and len(results) > 0:
            # Iterate through each detected object
            for obj in results[0]:
                confidence = obj[4].item() if len(obj) > 4 and obj[4].numel() > 0 else 0.0
                class_id = int(obj[5].item()) if len(obj) > 5 else -1

                # Check if the detected class is a person
                if class_id == -1:  # Assuming "person" class index is 0
                    print("set person_detected to TRUE")
                    person_detected = True
                    break  # Stop checking other objects once a person is found
                else:
                    print(f"Detected object with class_id: {class_id}")

        else:
            print("No detections found.")
            print("Results structure:", results)

        cv2.imshow("Camera Detection", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit(0)  # Ensure proper exit if 'q' is pressed

    # Rotating if person detected
    twist = Twist()
    if person_detected:
        rotating = True
        twist.angular.z = 0.5  # Adjust rotation speed as necessary
        print("Rotating because person detected")

        cmd_vel_pub.publish(twist)  # Publish the stop command
        #time.sleep(1.0)  # Wait for one second
        print("Just after the if person_detected: block")  # Add this line

    else:
        rotating = False
        #twist.angular.z = 0.0
        print("start obstacle avoidance!!")
        # Laser scan processing
        if s1_distance >= 0.4:
            twist.linear.x = 0.1  # Adjust linear speed as necessary
            print("##############")
            print("Moving forward")
        else:
            twist.angular.z = 0.5
            print("##############")
            print("Rotating due to laser scan")

        if 0 < s4_distance <= 0.3:
            twist.angular.z = 0.5
            print("##############")
            print("Rotating due to laser scan (s4)")

    cmd_vel_pub.publish(twist)

def main(args=None):
    print('START!!!')
    rclpy.init(args=args)
    node = rclpy.create_node('laser_data')
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    rotating = False

    # Initialize camera and YOLO model
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(3, 160)  # Set width
    cap.set(4, 120)  # Set height
    model = YOLO("yolo-Weights/yolov5nu.pt")

    def laser_callback(msg):
        nonlocal rotating
        callback(msg, cmd_vel_pub, rotating, model, cap)

    sub = node.create_subscription(LaserScan, 'scan', laser_callback, rclpy.qos.qos_profile_sensor_data)
    rclpy.spin(node)

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
