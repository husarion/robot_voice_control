
from orders_supervisor.msg import OrderGoal
from converter.math import planar_to_posestamped
from converter.CaseWrapper import CaseWrapper


class Converter(object):
    def __init__(self, order_structure):
        self.order_structure = order_structure
        self.order_description = self.order_structure.get("description", self.order_structure.voice_alias)

        self.case_wrapper = CaseWrapper(order_structure)
        self.special_keys = ("voice_alias", "description", "label", "cases", "frame", "x", "y", "yaw")
    
    def __call__(self, txt):
        return self.get_order(txt)

    def get_order(self, txt):
        order = OrderGoal()
        case = self.case_wrapper(txt)
        if ("cases" in self.order_structure and not case):
            return "could not match any order with description: {} to txt: {}".format(self.order_description, txt)
        
        self._append_label(order, case)
        self._append_description(order, case)
        self._append_target_pose(order, case)
        self._append_params(order, case)

        return order 

    def _append_label(self, order, case):
        order.label = self.order_structure.label

    def _append_target_pose(self, order, case):
        order.target_pose = planar_to_posestamped(
                    case.get('x', 0), case.get('y', 0), case.get('yaw', 0), self.order_structure.get('frame', "")
        )


    def _append_params(self, order, case):
        pairwise_append = lambda item, lists: (lists[0].append(item[0]), lists[1].append(item[1]))  
        def append(item):
            key, val = item
            if not key in self.special_keys:
                if type(val) is str: pairwise_append(item, (order.skeys, order.svals))
                else: pairwise_append(item, (order.fkeys, order.fvals))                    

        map(append, self.order_structure.items()) 
        map(append, case.items()) 
                

    def _append_description(self, order, case):
        order.description = " ".join((self.order_description, case.get("description", case.voice_alias))) \
                                    if case else self.order_description

