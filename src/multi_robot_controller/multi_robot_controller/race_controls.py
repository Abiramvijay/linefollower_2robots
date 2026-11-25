#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class RaceControls(Node):
    def __init__(self):
        super().__init__('race_controls')
        self.start_publisher = self.create_publisher(Bool, '/start_race', 10)
        self.return_publisher = self.create_publisher(Bool, '/return_home', 10)

    def publish_start(self, start: bool) -> None:
        self.start_publisher.publish(Bool(data=start))
        message = 'Start' if start else 'Stop'
        self.get_logger().info(f'{message} command sent to both robots.')

    def publish_return_home(self) -> None:
        self.return_publisher.publish(Bool(data=True))
        self.get_logger().info('Return-home command sent to both robots.')


def main(args=None):
    import tkinter as tk

    rclpy.init(args=args)
    node = RaceControls()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    root = tk.Tk()
    root.title('Multi-Robot Line Following')
    root.resizable(False, False)
    root.protocol('WM_DELETE_WINDOW', root.destroy)

    frame = tk.Frame(root, padx=20, pady=16)
    frame.pack()
    tk.Label(
        frame,
        text='Both robots are ready at the start of their paths.',
        font=('TkDefaultFont', 11),
    ).pack(pady=(0, 12))
    tk.Button(
        frame,
        text='START BOTH ROBOTS',
        command=lambda: node.publish_start(True),
        bg='#198754',
        fg='white',
        activebackground='#146c43',
        font=('TkDefaultFont', 12, 'bold'),
        padx=18,
        pady=10,
    ).pack(fill='x', pady=4)
    tk.Button(
        frame,
        text='STOP BOTH ROBOTS',
        command=lambda: node.publish_start(False),
        bg='#dc3545',
        fg='white',
        activebackground='#b02a37',
        font=('TkDefaultFont', 11, 'bold'),
        padx=18,
        pady=8,
    ).pack(fill='x', pady=4)
    tk.Button(
        frame,
        text='RETURN BOTH ROBOTS HOME',
        command=node.publish_return_home,
        bg='#0d6efd',
        fg='white',
        activebackground='#0a58ca',
        font=('TkDefaultFont', 11, 'bold'),
        padx=18,
        pady=8,
    ).pack(fill='x', pady=4)

    try:
        root.mainloop()
    finally:
        node.publish_start(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
