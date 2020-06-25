import re
from common import is_expression_in_string
from geometry_msgs.msg import PoseStamped
from orders_supervisor.msg import OrderGoal, Orders
from converter.Converter import Converter
from bunch import bunchify

class TxtToOrders:
    def __init__(self, separator, orders):
        self.separator = separator
        self.possible_orders = bunchify(orders)
        self.converters = {}
        self._bind_converters()
    
    def __call__(self, text):
        txt_orders = self._separate(text)
        data = map(lambda txt: self._txt_to_order(txt), txt_orders)
        data, errors = self._split_errors(data)
        orders = Orders()
        orders.data = data
        return orders, errors

    def _bind_converters(self):
        for order_name, order in self.possible_orders.items():
            self.converters[order_name] = Converter(order)

    def _txt_to_order(self, txt):
        for order in self.possible_orders:
            if self._is_order_in_string(self.possible_orders[order], txt):
                converter = self.converters[order]
                return converter(txt)

        return "could not match txt: {} to any order".format(txt)

    def _split_errors(self, orders_list):
        good_orders = []
        errors = []

        for order in orders_list:
            if type(order) is OrderGoal:
                good_orders.append(order)  
            else:
                 errors.append(order)

        return good_orders, errors

    def _separate(self, output):
       return re.split(self.separator, output.lower())

    def _is_order_in_string(self, order, string):
        return is_expression_in_string(order.voice_alias, string)
