#!/usr/bin/env python
import rospy 
from TxtToOrders import TxtToOrders
from rospkg import RosPack
from os.path import join as join_paths

from txt_to_orders.msg import Dictionary
from txt_to_orders.srv import TxtToOrdersSrv, TxtToOrdersSrvResponse
from Data import Data

class Main:
    def __init__(self):
        self._wait_for_params()
        
        self.rate = rospy.Rate(10)
        self.data = Data(rospy.get_param("/orders_list"))
        self.txt_to_orders = TxtToOrders(self.data.separator, self.data.orders)
        
        self.dict_publisher = rospy.Publisher("~dictionary", Dictionary, queue_size=1) 
        self.txt_to_orders_srv = rospy.Service("~txt_to_orders", TxtToOrdersSrv, self.convert)

    def convert(self, req):
        resp = TxtToOrdersSrvResponse()
        if req.text != "":
            resp = TxtToOrdersSrvResponse()
            resp.orders, resp.errors = self.txt_to_orders(req.text)
        else:
            resp.errors = ["empty txt"]          

        return resp

    def run(self):
        while not rospy.is_shutdown():
            self._publish_dict()
    
    def _publish_dict(self):
        self.dict_publisher.publish(self.data.dictionary)
        self.rate.sleep()

    def _wait_for_params(self):
        r = rospy.Rate(0.2)
        while not (rospy.has_param("/orders_list") or rospy.is_shutdown()):
            rospy.loginfo("[txt_to_orders] Waiting for order executor")
            r.sleep()




if __name__ == "__main__":
    rospy.init_node("txt_to_orders")
    main = Main()
    main.run()

    

        
