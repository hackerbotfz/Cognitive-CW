import rclcpp
from rclcpp.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import os
import math

class LandmarkDatabase(Node):
    def __init__(self):
        super().__init__('landmark_database')

        # Parameters
        self.declare_parameter('database_file', 'landmark_db.json')
        self.declare_parameter('distance_threshold', 0.5) # Meters to consider a landmark "new"

        self.db_file = self.get_parameter('database_file').get_parameter_value().string_value
        self.dist_thresh = self.get_parameter('distance_threshold').get_parameter_value().double_value

        # Storage: {id: {'x': val, 'y': val, 'z': val, 'type': label, 'count': n}}
        self.landmarks = {}
        
        # Load existing db if available
        self.load_database()

        # Subscribers
        # Assuming object detection publishes visualization markers
        self.subscription = self.create_subscription(
            MarkerArray,
            '/detected_markers',
            self.marker_callback,
            10)

        # Publishers
        self.db_publisher = self.create_publisher(MarkerArray, '/landmark_database_vis', 10)
        
        # Timer for saving/publishing
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Landmark Database Node Started')

    def load_database(self):
        if os.path.exists(self.db_file):
            try:
                with open(self.db_file, 'r') as f:
                    self.landmarks = json.load(f)
                self.get_logger().info(f'Loaded {len(self.landmarks)} landmarks from Json.')
            except Exception as e:
                self.get_logger().error(f'Failed to load database: {str(e)}')

    def save_database(self):
        try:
            with open(self.db_file, 'w') as f:
                json.dump(self.landmarks, f, indent=4)
        except Exception as e:
            self.get_logger().error(f'Failed to save database: {str(e)}')

    def is_duplicate(self, pos):
        # Check against existing landmarks
        for lm_id, data in self.landmarks.items():
            dist = math.sqrt(
                (data['x'] - pos.x)**2 + 
                (data['y'] - pos.y)**2 + 
                (data['z'] - pos.z)**2
            )
            if dist < self.dist_thresh:
                return lm_id # Return the ID of the close match
        return None

    def marker_callback(self, msg):
        for marker in msg.markers:
            # We assume the marker ID or text identifies the class (e.g. "stop_sign")
            # If your detection node provides a specific ID, use it.
            # Here we generate a unique ID based on position if it's new.
            
            existing_id = self.is_duplicate(marker.pose.position)
            
            if existing_id:
                # Update existing landmark (simple moving average for stability)
                lm = self.landmarks[existing_id]
                n = lm['count']
                lm['x'] = (lm['x'] * n + marker.pose.position.x) / (n + 1)
                lm['y'] = (lm['y'] * n + marker.pose.position.y) / (n + 1)
                lm['z'] = (lm['z'] * n + marker.pose.position.z) / (n + 1)
                lm['count'] += 1
                self.get_logger().debug(f'Updated landmark {existing_id}')
            else:
                # Add new landmark
                new_id = str(len(self.landmarks) + 1)
                self.landmarks[new_id] = {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z,
                    'type': marker.text if marker.text else f"landmark_{new_id}",
                    'count': 1
                }
                self.get_logger().info(f'Added new landmark {new_id} at ({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f})')

    def timer_callback(self):
        self.save_database()
        self.publish_database_markers()

    def publish_database_markers(self):
        ma = MarkerArray()
        for lm_id, data in self.landmarks.items():
            m = Marker()
            m.header.frame_id = "map" # Assuming landmarks are in map frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = int(lm_id)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = data['x']
            m.pose.position.y = data['y']
            m.pose.position.z = data['z']
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.a = 1.0
            m.color.g = 1.0 # Green for database objects
            m.text = data['type']
            
            # Text label
            text_marker = Marker()
            text_marker.header = m.header
            text_marker.id = int(lm_id) + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = data['x']
            text_marker.pose.position.y = data['y']
            text_marker.pose.position.z = data['z'] + 0.3
            text_marker.scale.z = 0.2
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"{data['type']} ({lm_id})"

            ma.markers.append(m)
            ma.markers.append(text_marker)
        
        self.db_publisher.publish(ma)

def main(args=None):
    rclcpp.init(args=args)
    node = LandmarkDatabase()
    try:
        rclcpp.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_database()
        node.destroy_node()
        rclcpp.shutdown()

if __name__ == '__main__':
    main()
