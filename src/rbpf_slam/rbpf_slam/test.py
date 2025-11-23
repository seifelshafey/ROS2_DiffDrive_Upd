import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

# CONFIGURATION
BAG_PATH = './my_slam_data' 
TOPIC = '/odometry/filtered'

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
reader.open(storage_options, converter_options)

prev_x = None
count = 0

print(f"--- Inspecting {TOPIC} ---")

while reader.has_next():
    (topic, data, t) = reader.read_next()
    
    if topic == TOPIC:
        msg = deserialize_message(data, Odometry)
        curr_x = msg.pose.pose.position.x
        
        if prev_x is not None:
            diff = curr_x - prev_x
            
            # Check if truly identical (floating point equality)
            if diff == 0.0:
                print(f"Frame {count}: EXACT DUPLICATE")
            else:
                # Print with high precision
                # .20f = 20 decimal places
                # .5e  = Scientific notation (good for tiny numbers)
                print(f"Frame {count} | Diff: {diff:.5e} | Curr: {curr_x:.10f}")
                
        prev_x = curr_x
        count += 1
        
        if count > 2000: break