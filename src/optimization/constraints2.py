from ortools.sat.python import cp_model

from src.core.entities import Scenario
from src.optimization.var_manager import VarManager


class ConstraintFactory:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario, var_manager: VarManager):
        self.model = model
        self.var_manager = var_manager
        self.scenario = scenario
    def _add_load_flow_constraints(self):
        CHECK_CONSERVATION = True
        for v in self.scenario.vehicles:


            if CHECK_CONSERVATION:
                for i in self.scenario.all_locations:
                    for j in self.scenario.all_locations:
                        x_kij = self.var_manager.get_routing_var(v.id, i.id, j.id)
                        if x_kij is None: continue
                        self.model.add(
                            self.var_manager.get_load_arriving_var(v.id, j.id) ==
                            self.var_manager.get_load_at_point_var(v.id, i.id)
                        ).OnlyEnforceIf(x_kij)
            for loc in self.scenario.all_locations:
                is_loc_visited = self.var_manager.get_is_visited_var(v.id, loc.id)
                if is_loc_visited is None: continue
                consumed = sum(self.var_manager.get_delivery_var(v.id, loc.id, b.id) for b in self.scenario.brands)
                # Временно игнорируем pickup для простоты
                if CHECK_CONSERVATION:
                    self.model.add(
                        self.var_manager.get_load_at_point_var(v.id, loc.id) ==
                        self.var_manager.get_load_arriving_var(v.id, loc.id) - consumed
                    ).OnlyEnforceIf(is_loc_visited)
