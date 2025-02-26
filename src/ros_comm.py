#!/usr/bin/env python3
#import rospy
import socket
import threading
import time
from ros_comm_tasks import Task
from ros_object_detections import serialize_detections, detect_objects

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 65432        # The port used by the server

def server():
    """Single-threaded socket server for ROS communication."""
    # rospy.init_node("LLM_agent")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"ROS Communication Server listening on {HOST}:{PORT}")

        while True:
            conn, addr = s.accept()
            print(f"Connected by {addr}")

            with conn:
                while True:  # Keep connection open for multiple messages
                    try:
                        print("Waiting for data...")
                        data = conn.recv(1024).decode().strip()

                        if not data:
                            print(f"Client {addr} disconnected.")
                            break  # wait for client to send message

                        print(f"Received: {data}")
                        response = process_task(data)
                        conn.sendall(response.encode())

                    except ConnectionResetError:
                        print(f"Connection lost with {addr}. Waiting for a new connection...")
                        break  # Exit inner loop and accept new client

                    time.sleep(0.5)
                    # rate.sleep()

def process_task(data):
    """Processes incoming tasks and returns appropriate responses."""
    parts = data.split(":", 1)  # Split into task and data
    
    if len(parts) != 2:
        return "Invalid message format"
    
    task, payload = parts
    response = ""

    if task == Task.GET_OBJECT_NAMES.value: 
        response = serialize_detections(detect_objects())
    elif task == Task.GET_OBJECTS.value:
        response = "Mock object list"
    else:
        response = "Unknown task"

    return response

def robot_execute(task, message):
    """Client function to send a request and receive response."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT))
            full_message = f"{task}:{message}"
            s.sendall(full_message.encode())
            data = s.recv(1024).decode()
            return data
        except ConnectionError:
            return "Connection to server failed"


if __name__ == "__main__":
    ros_comm_thread = threading.Thread(target=server)
    ros_comm_thread.daemon = True # Allow the program to exit even if this thread is running.
    ros_comm_thread.start()
    time.sleep(1) # Give the server a moment to start.
    print(robot_execute(Task.GET_OBJECTS.value, ""))
