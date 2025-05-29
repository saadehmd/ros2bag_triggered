#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox, CheckButtons
import numpy as np
import time
import threading


class DummyTrigger:

    def __init__(self, node, topic_name, msg_type, trigger_name):
        self.topic_name = topic_name
        self.publisher = None
        self.publish_freq = 1.0
        self.trigger_name = trigger_name
        self.activated = False
        self.publisher = node.create_publisher(msg_type, topic_name, 10)
        self.timer = node.create_timer(1.0 / self.publish_freq, self.publish_trigger)
        self.publishing = False

    def publish_trigger(self):
        if self.publishing:
            this_msg = self.get_positive_msg() if self.activated else self.get_negative_msg()   
            print(f"Publishing {this_msg} on {self.topic_name}")
            self.publisher.publish(this_msg)

    def get_positive_msg(self):
        raise NotImplementedError("Subclasses should implement this method")
    
    def get_negative_msg(self):
        raise NotImplementedError("Subclasses should implement this method")

    
class DummyTriggerUI(Node):

    def __init__(self):

        super().__init__('dummy_trigger_ui')
        self.triggers = []
        self.activate_buttons = []
        self.deactivate_buttons = []
        self.freq_textboxes = []
        self.publish_buttons = []

    def add_triggers(self, triggers):
        for trigger in triggers:
            if isinstance(trigger, DummyTrigger):
                self.triggers.append(trigger)
            else:
                raise TypeError(f"Expected DummyTrigger instance, got {type(trigger)}")

    def run(self):

        try:
            self.create_ui()

            for trigger in self.triggers:
                self.add_trigger_display(trigger)

            ros_thread = threading.Thread(target=self.spin_ros)
            ros_thread.start()
            plt.show()
            ros_thread.join(timeout=1.0)
        except KeyboardInterrupt:
            print("KeyboardInterrupt received, shutting down Dummy trigger UI")

        except Exception as e:
            print(f"An error occurred: {e}")

    
    def spin_ros(self):

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
        except Exception as e:
            print(f"An error occurred while spinning Trigger UI: {e}")
            plt.close('all')
            raise e

    def create_ui(self):
        self.fig, self.ax = plt.subplots(figsize=(len(self.triggers), 8))
        self.fig.canvas.manager.set_window_title('ROS2 Trigger Generator')
        
        plt.subplots_adjust(left=0.1, right=0.9, top=0.95, bottom=0.05)
        self.ax.set_axis_off()
        
        self.ax.text(0.5, 0.98, 'ROS2 Trigger Generator', 
                    horizontalalignment='center', fontsize=16, weight='bold')

    def add_trigger_display(self, trigger):
        # Calculate a unique y-offset for each trigger based on its index
        idx = self.triggers.index(trigger)
        y_offset_title = 0.9 - idx * 0.1
        y_offset = 0.85 - idx * 0.1
        y_offset_console = 0.9 - len(self.triggers) * 0.1

        ax_trigger = self.ax.text(0.05, y_offset_title, trigger.trigger_name, 
                                  horizontalalignment='center', fontsize=12)

        ax_positive_button = plt.axes([0.2,y_offset, 0.15, 0.05])
        ax_negative_button = plt.axes([0.4,y_offset, 0.15, 0.05])
        ax_freq_textbox = plt.axes([0.6,y_offset, 0.05, 0.05])
        ax_publish_button = plt.axes([0.7,y_offset, 0.05, 0.05])

        self.activate_buttons.append(Button(ax_positive_button, 'Activate', hovercolor='lightgreen'))
        self.deactivate_buttons.append(Button(ax_negative_button, 'Deactivate', hovercolor='pink'))
        self.freq_textboxes.append(TextBox(ax_freq_textbox, 'Freq (Hz)', initial=str(trigger.publish_freq)))
        self.publish_buttons.append(Button(ax_publish_button, 'Publish', hovercolor='lightblue'))
        self.console_log = self.ax.text(0.5, y_offset_console, '',
                                       horizontalalignment='center', fontsize=10,
                                       bbox=dict(facecolor='lightgreen', alpha=0.5))


        def activate(event):
            trigger.activated = True
            self.activate_buttons[idx].color = 'green'
            self.activate_buttons[idx].hovercolor = 'green'
            self.deactivate_buttons[idx].color = 'grey'
            self.deactivate_buttons[idx].hovercolor = 'pink'
            self.fig.canvas.draw_idle()
            print(f"{trigger.trigger_name} activated")
            self.console_log.set_text(f"{trigger.trigger_name} activated")
            self.console_log.set_bbox(dict(facecolor='lightgreen', alpha=0.5))

        def deactivate(event):
            trigger.activated = False
            self.deactivate_buttons[idx].color = 'red'
            self.deactivate_buttons[idx].hovercolor = 'red'
            self.activate_buttons[idx].color = 'grey'
            self.activate_buttons[idx].hovercolor = 'lightgreen'
            self.fig.canvas.draw_idle()
            print(f"{trigger.trigger_name} deactivated")
            self.console_log.set_text(f"{trigger.trigger_name} deactivated")
            self.console_log.set_bbox(dict(facecolor='lightgreen', alpha=0.5))

        self.activate_buttons[idx].on_clicked(activate)
        self.deactivate_buttons[idx].on_clicked(deactivate)

        self.freq_textboxes[idx].on_submit(lambda text: self.update_frequency(text, trigger))
        self.publish_buttons[idx].on_clicked(lambda event: self.update_publish_status(event, trigger))

    def update_frequency(self, text, trigger):

        try:
            new_freq = float(text)
            if new_freq <= 0:
                self.console_log.set_text('Error: Frequency must be positive')
                self.console_log.set_bbox(dict(facecolor='red', alpha=0.5))
                print(f"Error: Frequency must be positive")
                self.fig.canvas.draw_idle()
                return
            else:    
                trigger.publish_freq = new_freq
                if trigger.timer is not None:
                    trigger.timer.cancel()
                trigger.timer = self.create_timer(1.0 / new_freq, trigger.publish_trigger)
                self.console_log.set_text(f"{trigger.trigger_name} frequency set to {new_freq} Hz")
                self.console_log.set_bbox(dict(facecolor='lightgreen', alpha=0.5))
                print(f"{trigger.trigger_name} frequency set to {new_freq} Hz")

        except ValueError:
            self.console_log.set_text('Error: Invalid frequency value')
            self.console_log.set_bbox(dict(facecolor='red', alpha=0.5))
            print(f"Error: Invalid frequency value")
            self.fig.canvas.draw_idle()

    def update_publish_status(self, event, trigger):
        trigger.publishing = not trigger.publishing
        if trigger.publishing:
            #trigger.timer = self.create_timer(1.0 / trigger.publish_freq, trigger.publish_trigger)
            print(f"{trigger.trigger_name} publishing started")
            self.console_log.set_text(f"{trigger.trigger_name} publishing started")
            self.console_log.set_bbox(dict(facecolor='lightgreen', alpha=0.5))
        else:
            #if trigger.timer is not None:
            #    trigger.timer.cancel()
            print(f"{trigger.trigger_name} publishing stopped")
            self.console_log.set_text(f"{trigger.trigger_name} publishing stopped")
            self.console_log.set_bbox(dict(facecolor='lightgreen', alpha=0.5))
        self.fig.canvas.draw_idle()
        idx = self.triggers.index(trigger)
        self.publish_buttons[idx].color = 'blue' if trigger.publishing else 'grey'
        self.publish_buttons[idx].hovercolor = 'blue' if trigger.publishing else 'lightblue'