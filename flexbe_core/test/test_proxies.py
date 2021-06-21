#!/usr/bin/env python
import unittest
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached, ProxyActionClient, ProxyServiceCaller

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from flexbe_msgs.msg import BehaviorExecutionAction, BehaviorExecutionGoal, BehaviorExecutionResult


class TestProxies(unittest.TestCase):

    def test_publish_subscribe(self):
        t1 = '/pubsub_1'
        t2 = '/pubsub_2'
        pub = ProxyPublisher({t1: String})
        pub = ProxyPublisher({t2: String}, _latch=True)
        sub = ProxySubscriberCached({t1: String})
        self.assertTrue(pub.is_available(t1))

        self.assertTrue(pub.wait_for_any(t1))
        self.assertFalse(pub.wait_for_any(t2))
        pub.publish(t1, String('1'))
        pub.publish(t2, String('2'))

        rospy.sleep(.5)  # make sure latched message is sent before subscriber is added
        sub = ProxySubscriberCached({t2: String})
        rospy.sleep(.5)  # make sure latched message can be received before checking

        self.assertTrue(sub.has_msg(t1))
        self.assertEqual(sub.get_last_msg(t1).data, '1')
        sub.remove_last_msg(t1)
        self.assertFalse(sub.has_msg(t1))
        self.assertIsNone(sub.get_last_msg(t1))

        self.assertTrue(sub.has_msg(t2))
        self.assertEqual(sub.get_last_msg(t2).data, '2')

    def test_subscribe_buffer(self):
        t1 = '/buffered_1'
        pub = ProxyPublisher({t1: String})
        sub = ProxySubscriberCached({t1: String})
        sub.enable_buffer(t1)
        self.assertTrue(pub.wait_for_any(t1))

        pub.publish(t1, String('1'))
        pub.publish(t1, String('2'))
        rospy.sleep(.5)  # make sure messages can be received

        self.assertTrue(sub.has_msg(t1))
        self.assertTrue(sub.has_buffered(t1))
        self.assertEqual(sub.get_from_buffer(t1).data, '1')

        pub.publish(t1, String('3'))
        rospy.sleep(.5)  # make sure messages can be received

        self.assertEqual(sub.get_from_buffer(t1).data, '2')
        self.assertEqual(sub.get_from_buffer(t1).data, '3')
        self.assertIsNone(sub.get_from_buffer(t1))
        self.assertFalse(sub.has_buffered(t1))

    def test_service_caller(self):
        t1 = '/service_1'
        rospy.Service(t1, Trigger, lambda r: (True, 'ok'))

        srv = ProxyServiceCaller({t1: Trigger})

        result = srv.call(t1, TriggerRequest())
        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertEqual(result.message, 'ok')

        self.assertFalse(srv.is_available('/not_there'))
        srv = ProxyServiceCaller({'/invalid': Trigger}, wait_duration=.1)
        self.assertFalse(srv.is_available('/invalid'))

    def test_action_client(self):
        t1 = '/action_1'
        server = None

        def execute_cb(goal):
            rospy.sleep(.5)
            if server.is_preempt_requested():
                server.set_preempted()
            else:
                server.set_succeeded(BehaviorExecutionResult(outcome='ok'))

        server = actionlib.SimpleActionServer(t1, BehaviorExecutionAction, execute_cb, auto_start=False)
        server.start()

        client = ProxyActionClient({t1: BehaviorExecutionAction})
        self.assertFalse(client.has_result(t1))
        client.send_goal(t1, BehaviorExecutionGoal())

        rate = rospy.Rate(20)
        for i in range(20):
            self.assertTrue(client.is_active(t1) or client.has_result(t1))
            rate.sleep()
        self.assertTrue(client.has_result(t1))

        result = client.get_result(t1)
        self.assertEqual(result.outcome, 'ok')

        client.send_goal(t1, BehaviorExecutionGoal())
        rospy.sleep(.1)
        client.cancel(t1)
        rospy.sleep(.9)

        self.assertFalse(client.is_active(t1))

        self.assertFalse(client.is_available('/not_there'))
        client = ProxyActionClient({'/invalid': BehaviorExecutionAction}, wait_duration=.1)
        self.assertFalse(client.is_available('/invalid'))


if __name__ == '__main__':
    rospy.init_node('test_flexbe_proxies')
    import rostest
    rostest.rosrun('flexbe_core', 'test_flexbe_proxies', TestProxies)
