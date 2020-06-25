import re

def is_expression_in_string(exp, string):
    return (re.search(exp, string) is not None)

