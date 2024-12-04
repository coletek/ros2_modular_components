import time
from functools import partial
import rclpy
from rclpy.node import Node
from custom_messages.msg import *
from std_srvs.srv import SetBool

# future
#from std_msgs.msg import Float64
#from control_msgs.msg import JointTrajectoryControllerFeedback

class ControllerDualActuator(Node):

    Kp = 0.5
    Ki = 0.0 #0.1
    Kd = 0.0 #0.01
    max_speed = 1.0
    period = 0.1 # 10Hz - must equal the sensor data - 100Hz might be better
    _last_error = {}
    _error_sum = {}
    _motor_speed_previous = {}
    
    _desired_positions = {}
    _actual_positions = {}
    _dual_actuators = {}
    _pubs = {}
    _subs = {}

    _timer = None
    _is_active = False
    
    def __init__(self):
        super().__init__('controller_dual_actuator')

        self._freq_hz = 1.0/self.period

        self.declare_parameter("p_gain", 1.0)
        self.declare_parameter("i_gain", 0.1)
        self.declare_parameter("d_gain", 0.01)
        self.declare_parameter("hysteresis", 0.001)
        
        self.declare_parameter("sub", "/controller_dual_actuator/set_point")
        self.declare_parameter("pub", "/controller_dual_actuator/feedback")
        self.declare_parameter('subs_sensors', [ '/sensor_feedback/ch0', '/sensor_feedback/ch1' ])
        self.declare_parameter('pubs_motors', [ '/device_motor/ch0', '/device_motor/ch1' ])
        self.declare_parameter("srv", "/controller_dual_actuator/enable" )

        for i in range(2):
            sub = self.get_parameter('subs_sensors').value[i]
            pub = self.get_parameter('pubs_motors').value[i]
            self._dual_actuators[pub] = {
                "sub_sensor": sub,
                "pub_motor": pub,
            }
            self._subs[sub] = pub
            self._desired_positions[pub] = 0.0
            self._actual_positions[pub] = 0.0
            self._last_error[pub] = 0.0
            self._error_sum[pub] = 0.0
            self._motor_speed_previous[pub] = 0.0
            self.get_logger().info("Creating sub '%s' and pub '%s'" % (sub, pub))
            self.create_subscription(PercentageMessage, sub, partial(self.callback_sensor_feedback, name=sub), 10)
            self._pubs[pub] = self.create_publisher(PWMMessage, pub, 10)
            i += 1

        self._pub = self.create_publisher(ControllerFeedbackMessage, self.get_parameter('pub').value, 10)
            
        # input UI
        self._sub = self.get_parameter("sub").value
        self.create_subscription(PercentageMessage, self._sub, self.callback_sp, 10)
        # TODO - future method
        #self.subscriber = self.create_subscription(JointTrajectoryControllerFeedback, 'actuator/sp', self.actuator_sp_callback, 10)

        self.create_service(SetBool, self.get_parameter('srv').value, self.callback_enable)
        
    def callback_enable(self, request, response):
        if request.data:
            response.success = True
            if self._is_active:
                response.success = False
                response.message = "Already started"
            else:
                response.success = True
                response.message = "Starting timer"
                self._timer = self.create_timer(1.0/self._freq_hz, self.callback_timer)
                self._is_active = True
        else:
            if self._is_active:
                self._timer.cancel()
                self._is_active = False
                self.disable()
                response.success = True
                response.message = "Stopping timer"
            else:
                response.success = False
                response.message = "Already disabled"
            
        self.get_logger().info('Incoming enable(%r): %r (%s)' % (request.data, response.success, response.message))
        return response
    
    def callback_sensor_feedback(self, msg, name):
        self.get_logger().debug("received '%s': %s%%" % (name, msg.percent))
        self._actual_positions[self._subs[name]] = float(msg.percent)

    def callback_sp(self, msg):
        self.get_logger().info('received sp: %s%%' % msg.percent)
        for key, val in self._pubs.items():
            self._desired_positions[key] = float(msg.percent)

    def callback_timer(self):
    #def actuator_sp_callback(self, msg):
        # Calculate error between desired position and current position

        error = {}
        error_diff = {}
        motor_speed = {}
        
        for key, val in self._pubs.items():
            
            # Calculate error between desired position and current position
            error[key] = self._desired_positions[key] - self._actual_positions[key]
            self.get_logger().info("%s: desired=%f actual=%f" % (key, self._desired_positions[key], self._actual_positions[key]))
            #TODO future: error[key] = msg.desired.positions[key] - msg.actual.positions[key]

            # Calculate error derivatives
            error_diff[key] = (error[key] - self._last_error[key]) / self.period

            # Calculate error integrals
            self._error_sum[key] += error[key] * self.period

            # Calculate motor speeds
            motor_speed[key] = self.Kp * error[key] + self.Ki * self._error_sum[key] + self.Kd * error_diff[key]

            # Limit motor speeds to maximum values
            motor_speed[key] = max(min(motor_speed[key], self.max_speed), -self.max_speed)

            # Publish feedback
            msg = ControllerFeedbackMessage()
            msg.name = key
            msg.sp = self._desired_positions[key]
            msg.op = self._actual_positions[key]
            msg.error = error[key]
            msg.is_completed = False
            if abs(error[key]) < self.get_parameter('hysteresis').value:
                msg.is_completed = True
            self._pub.publish(msg)
            self.get_logger().info('published control_feedback (%s): %f (%r)' %
                                   (key, msg.error, msg.is_completed))
            
            # Publish motor speeds if they have changed
            if motor_speed[key] != self._motor_speed_previous[key]:
                msg = PWMMessage()
                msg.duty_cycle = motor_speed[key]
                val.publish(msg)
                #self._pubs[key].publish(PWMMessage(motor_speed[key]))
                self.get_logger().info('published device_motor (%s): %s%%' % (key, msg.duty_cycle))

            self._motor_speed_previous[key] = motor_speed[key]
                
        # Update error derivatives
        self._last_error[key] = error[key]

    def disable(self):
        self.get_logger().info('disable')
        for key, val in self._pubs.items():
            msg = PWMMessage()
            msg.duty_cycle = 0.0
            val.publish(msg)
            self.get_logger().info('published device_motor (%s): %s%%' % (key, msg.duty_cycle))
        
def main(args=None):

    rclpy.init(args=args)

    node = ControllerDualActuator()

    rclpy.spin(node)
    # Set the target height
    #node.target_height = 10.0
    # Control the actuators until target height is reached
    #while not (abs(node.actuator1_pos - node.target_height) < 0.1 and abs(node.actuator2_pos - node.target_height) < 0.1):
    #    node.spin_once()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
