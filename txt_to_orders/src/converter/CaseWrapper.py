from common import is_expression_in_string

class CaseWrapper:
    def __init__(self, order_structure):
        self.cases_structure = order_structure.get("cases")

    def __call__(self, txt):
        if self.cases_structure is None:
            return dict() #return dict to make it possible to call .get method
        
        case_key = self._match_case(txt)
        case = self.cases_structure.get(case_key, dict())

        return case

    def _match_case(self, txt):
        check_matching = lambda case: \
            is_expression_in_string(self.cases_structure[case].voice_alias, txt)

        matching_cases = filter(check_matching, self.cases_structure)
        return next(iter(matching_cases), None)
