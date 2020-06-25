import os
from rospkg import RosPack
import yaml
import exrex
import re

from txt_to_orders.msg import Dictionary

class Data:
    def __init__(self, orders):
        self.dictionary = None
        self.separator = None
        self.orders = None

        self.orders = orders["orders"]
        self.separator = orders["separator"]
        self._create_dictionary()


    def _create_dictionary(self):
        self.dictionary = Dictionary()
        data = []
        [self._add_order_voice_aliases(self.orders[order], data) for order in self.orders]
        data.append(self.separator)
        self.dictionary.data = list(set(data)) #drop duplicates

    #we need recursion here because cases (which are nested inside order) have their own voice aliases
    def _add_order_voice_aliases(self, order, dictionary):
        if type(order) is not dict:
            return

        for key in order:
            if key == "voice_alias":
                aliases = self._list_all_aliases(order["voice_alias"])
                
                dictionary.extend(aliases)
            else:
                self._add_order_voice_aliases(order[key], dictionary)        


    def _list_all_aliases(self, voice_alias):
        aliases = []
        for word in re.split(" ", voice_alias):
            aliases.extend(map(lambda x: str(x), (exrex.generate(word))))
        return aliases

