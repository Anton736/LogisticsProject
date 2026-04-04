from ortools.sat.python import cp_model
from typing import Optional, Set, Dict, Tuple

from src.optimization.pruner import RoutePruner
from src.core.entities import Scenario


class VarManager:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario, pruner: RoutePruner):
        self.model = model
        self.scenario = scenario
        self.pruner = pruner

        # 1. Получаем разрешенные пары один раз
        self.allowed_pairs = self.pruner.get_allowed_pairs()

        # 2. Определяем, какие локации вообще могут быть посещены (для фильтрации переменных)
        # Включаем все точки, которые участвуют в разрешенных дугах + все склады (как базы)
        self.reachable_locs: Set[str] = {loc_id for pair in self.allowed_pairs for loc_id in pair}
        for wh in self.scenario.warehouses:
            self.reachable_locs.add(wh.id)

        # Реестры переменных
        self.x: Dict[Tuple[int, str, str], cp_model.BoolVarT] = {}
        self.arrival_times: Dict[Tuple[int, str], cp_model.IntVar] = {}
        self.load_arriving: Dict[Tuple[int, str], cp_model.IntVar] = {}
        self.load_at_point: Dict[Tuple[int, str], cp_model.IntVar] = {}
        self.delivered_vol: Dict[Tuple[int, str, str], cp_model.IntVar] = {}
        self.pickup_vol: Dict[Tuple[int, str, str], cp_model.IntVar] = {}

        self.wh_active: Dict[str, cp_model.BoolVarT] = {}
        self.wh_max_vol: Dict[str, cp_model.IntVar] = {}
        self.wh_stock_change_per_visit: Dict[Tuple[str, str, int], cp_model.IntVar] = {}
        self.wh_visit_intervals: Dict[Tuple[str, int], cp_model.IntervalVar] = {}
        self.wh_visit_active_flags: Dict[Tuple[str, int], cp_model.BoolVarT] = {}

        self.vehicle_used: Dict[int, cp_model.BoolVarT] = {}
        self.total_dist: Dict[int, cp_model.IntVar] = {}
        self.total_time: Dict[int, cp_model.IntVar] = {}
        self.shift_start: Dict[int, cp_model.IntVar] = {}
        self.shift_end: Dict[int, cp_model.IntVar] = {}

        self._init_all_vars()

    def _init_all_vars(self):
        # Переменные машин
        for v in self.scenario.vehicles:
            v_id = v.id
            self.vehicle_used[v_id] = self.model.new_bool_var(f'used_v{v_id}')
            self.total_dist[v_id] = self.model.new_int_var(0, 1_000_000, f'dist_v{v_id}')
            self.total_time[v_id] = self.model.new_int_var(0, 1440, f'total_t_v{v_id}')
            self.shift_start[v_id] = self.model.new_int_var(0, 1440, f'start_v{v_id}')
            self.shift_end[v_id] = self.model.new_int_var(0, 1440, f'end_v{v_id}')

            # Переменные ТОЛЬКО для достижимых локаций
            for loc_id in self.reachable_locs:
                self.arrival_times[(v_id, loc_id)] = self.model.new_int_var(0, 1440, f'arr_v{v_id}_l{loc_id}')
                self.load_arriving[(v_id, loc_id)] = self.model.new_int_var(0, v.capacity,
                                                                            f'load_arr_v{v_id}_l{loc_id}')
                self.load_at_point[(v_id, loc_id)] = self.model.new_int_var(0, v.capacity,
                                                                            f'load_out_v{v_id}_l{loc_id}')

                for b in self.scenario.brands:
                    self.delivered_vol[(v_id, loc_id, b.id)] = self.model.new_int_var(0, v.capacity,
                                                                                      f'del_v{v_id}_l{loc_id}_b{b.id}')
                    self.pickup_vol[(v_id, loc_id, b.id)] = self.model.new_int_var(0, v.capacity,
                                                                                   f'pick_v{v_id}_l{loc_id}_b{b.id}')

            # Дуги только разрешенные прунером
            for i_id, j_id in self.allowed_pairs:
                self.x[(v_id, i_id, j_id)] = self.model.new_bool_var(f'x_v{v_id}_{i_id}_{j_id}')

        # Переменные складов
        for wh in self.scenario.warehouses:
            wh_id = wh.id
            self.wh_active[wh_id] = self.model.new_bool_var(f'wh_active_{wh_id}')
            self.wh_max_vol[wh_id] = self.model.new_int_var(0, 10_000_000, f'wh_max_flow_w{wh_id}')

            for v in self.scenario.vehicles:
                v_id = v.id
                v_active_at_wh = self.model.new_bool_var(f'wh_visit_active_w{wh_id}_v{v_id}')
                self.wh_visit_active_flags[(wh_id, v_id)] = v_active_at_wh

                self.wh_visit_intervals[(wh_id, v_id)] = self.model.new_optional_interval_var(
                    self.model.new_int_var(0, 1440, f'wh_start_w{wh_id}_v{v_id}'),
                    self.model.new_int_var(0, 1440, f'wh_dur_w{wh_id}_v{v_id}'),
                    self.model.new_int_var(0, 2880, f'wh_end_w{wh_id}_v{v_id}'),
                    v_active_at_wh,
                    f'wh_interval_w{wh_id}_v{v_id}'
                )

                for b in self.scenario.brands:
                    self.wh_stock_change_per_visit[(wh_id, b.id, v_id)] = self.model.new_int_var(
                        -v.capacity, v.capacity, f'stock_ch_w{wh_id}_b{b.id}_v{v_id}'
                    )

    # --- Безопасные геттеры (возвращают None если переменной нет) ---
    def get_routing_var(self, v_id: int, i: str, j: str) -> Optional[cp_model.BoolVarT]:
        return self.x.get((v_id, i, j))

    def get_arrival_var(self, v_id: int, loc_id: str) -> Optional[cp_model.IntVar]:
        return self.arrival_times.get((v_id, loc_id))

    def get_load_arriving_var(self, v_id: int, loc_id: str) -> Optional[cp_model.IntVar]:
        return self.load_arriving.get((v_id, loc_id))

    def get_load_at_point_var(self, v_id: int, loc_id: str) -> Optional[cp_model.IntVar]:
        return self.load_at_point.get((v_id, loc_id))

    def get_delivery_var(self, v_id: int, loc_id: str, brand_id: str) -> Optional[cp_model.IntVar]:
        return self.delivered_vol.get((v_id, loc_id, brand_id))

    def get_pickup_var(self, v_id: int, loc_id: str, brand_id: str) -> Optional[cp_model.IntVar]:
        return self.pickup_vol.get((v_id, loc_id, brand_id))

    def get_wh_active_var(self, wh_id: str) -> cp_model.BoolVarT:
        return self.wh_active[wh_id]

    def get_wh_max_vol_var(self, wh_id: str) -> cp_model.IntVar:
        return self.wh_max_vol[wh_id]

    def get_wh_stock_change_per_visit_var(self, wh_id: str, brand_id: str, v_id: int) -> cp_model.IntVar:
        return self.wh_stock_change_per_visit[(wh_id, brand_id, v_id)]

    def get_wh_visit_interval_var(self, wh_id: str, v_id: int) -> cp_model.IntervalVar:
        return self.wh_visit_intervals[(wh_id, v_id)]

    def get_wh_visit_active_flag(self, wh_id: str, v_id: int) -> cp_model.BoolVarT:
        return self.wh_visit_active_flags[(wh_id, v_id)]

    def get_vehicle_used_var(self, v_id: int) -> cp_model.BoolVarT:
        return self.vehicle_used[v_id]

    def get_total_dist_var(self, v_id: int) -> cp_model.IntVar:
        return self.total_dist[v_id]

    def get_total_time_var(self, v_id: int) -> cp_model.IntVar:
        return self.total_time[v_id]

    def get_shift_start_var(self, v_id: int) -> cp_model.IntVar:
        return self.shift_start[v_id]

    def get_shift_end_var(self, v_id: int) -> cp_model.IntVar:
        return self.shift_end[v_id]