# --- START OF FILE var_manager.py ---

from ortools.sat.python import cp_model
from typing import Dict, Tuple, List, Optional

from RouterPruner import RoutePruner
from entities import Scenario, Store, Warehouse


class VarManager:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario, pruner: RoutePruner):
        self.pruner = pruner
        self.model = model
        self.scenario = scenario

        # --- РЕЕСТРЫ ПЕРЕМЕННЫХ ---
        self.x = {}  # x_kij (движение: машина k из i в j)
        self.arrival_times = {}  # m_jt'k (время прибытия машины k в точку j)
        self.load_arriving = {} # load_arriving_k_j (загрузка ТС в ящиках при прибытии в точку j)
        self.load_at_point = {}  # k_iq (загрузка ТС в ящиках при выезде из точки i)

        # Объемы (m_lt'vk0Wb и m_lt'vk1Wb)
        self.delivered_vol = {}  # m_lt'vk0 (сколько привезли бренда b в точку l машиной k)
        self.pickup_vol = {}  # m_lt'vk1 (сколько забрали бренда b из точки l машиной k)

        # Складские переменные
        self.wh_active = {}  # w_I (используется ли склад)
        # wh_max_vol теперь будет представлять ОБЩИЙ ПОТОК (обработанный объем) для расчета стоимости склада
        self.wh_max_vol = {}  # w_iv (общий объем, прошедший через склад)

        # Переменные для ReservoirConstraint
        self.wh_stock_change_per_visit = {} # (wh_id, b_id, v_id) -> IntVar (delivered - pickedup)
        self.wh_visit_intervals = {} # (wh_id, v_id) -> IntervalVar (ОДИН интервал на ВИЗИТ машины на склад)
        self.wh_visit_active_flags = {} # (wh_id, v_id) -> BoolVar (активен ли визит машины на склад)

        # Итоги по машинам для целевой функции
        self.vehicle_used = {}  # w_I для машин
        self.total_dist = {}  # k_is'' (общий путь)
        self.total_time = {}  # k_it'' (время смены: m_kfinish - m_kstart)
        self.shift_start = {}  # m_kstart (время начала работы водителя)
        self.shift_end = {}  # m_kfinish (время окончания работы водителя)

        # Запуск инициализации
        self._init_all_vars()
        self.add_load_arriving_vars()

    def _init_all_vars(self):
        """
        Инициализация переменных с использованием фильтрации маршрутов (RoutePruner).
        """
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            self.vehicle_used[v.id] = self.model.NewBoolVar(f'used_v{v.id}')
            self.total_dist[v.id] = self.model.NewIntVar(0, 1_000_000, f'dist_v{v.id}')
            self.total_time[v.id] = self.model.NewIntVar(0, 1440, f'total_t_v{v.id}')
            self.shift_start[v.id] = self.model.NewIntVar(0, 1440, f'start_v{v.id}')
            self.shift_end[v.id] = self.model.NewIntVar(0, 1440, f'end_v{v.id}')

            for loc in self.scenario.all_locations:
                self.arrival_times[(v.id, loc.id)] = self.model.NewIntVar(0, 1440, f'arr_v{v.id}_l{loc.id}')
                self.load_at_point[(v.id, loc.id)] = self.model.NewIntVar(0, v.capacity, f'load_out_v{v.id}_l{loc.id}')

                for b in self.scenario.brands:
                    self.delivered_vol[(v.id, loc.id, b.id)] = self.model.NewIntVar(0, v.capacity, f'del_v{v.id}_l{loc.id}_b{b.id}')
                    self.pickup_vol[(v.id, loc.id, b.id)] = self.model.NewIntVar(0, v.capacity, f'pick_v{v.id}_l{loc.id}_b{b.id}')

            for i_id, j_id in allowed_pairs:
                self.x[(v.id, i_id, j_id)] = self.model.NewBoolVar(f'x_v{v.id}_{i_id}_{j_id}')

        for wh in self.scenario.warehouses:
            self.wh_active[wh.id] = self.model.NewBoolVar(f'wh_active_{wh.id}')
            self.wh_max_vol[wh.id] = self.model.NewIntVar(0, 10_000_000, f'wh_max_flow_w{wh.id}')

            for v in self.scenario.vehicles:
                # Один интервал на визит машины v на склад wh.
                # Он будет активен, если машина посещает склад.
                self.wh_visit_active_flags[(wh.id, v.id)] = self.model.NewBoolVar(f'wh_visit_active_w{wh.id}_v{v.id}')
                self.wh_visit_intervals[(wh.id, v.id)] = self.model.NewOptionalIntervalVar(
                    self.model.NewIntVar(0, 1440, f'wh_int_start_w{wh.id}_v{v.id}'), # start
                    self.model.NewIntVar(0, 1440, f'wh_int_dur_w{wh.id}_v{v.id}'),   # duration
                    self.model.NewIntVar(0, 2880, f'wh_int_end_w{wh.id}_v{v.id}'),   # end
                    self.wh_visit_active_flags[(wh.id, v.id)],                      # is_present
                    f'wh_visit_int_w{wh.id}_v{v.id}'
                )

                for b in self.scenario.brands:
                    # Изменение запаса для бренда B на складе WH, вызванное визитом машины V
                    self.wh_stock_change_per_visit[(wh.id, b.id, v.id)] = self.model.NewIntVar(
                        -v.capacity, v.capacity, f'stock_change_w{wh.id}_b{b.id}_v{v.id}'
                    )

    def add_load_arriving_vars(self):
        for v in self.scenario.vehicles:
            for loc in self.scenario.all_locations:
                self.load_arriving[(v.id, loc.id)] = self.model.NewIntVar(0, v.capacity, f"load_arr_v{v.id}_l{loc.id}")

    # --- Геттеры ---
    def get_routing_var(self, v_id: int, i: int, j: int) -> Optional[cp_model.BoolVar]:
        return self.x.get((v_id, i, j))
    def get_arrival_var(self, v_id: int, loc_id: int) -> cp_model.IntVar:
        return self.arrival_times[(v_id, loc_id)]
    def get_load_arriving_var(self, v_id: int, loc_id: int) -> cp_model.IntVar:
        return self.load_arriving[(v_id, loc_id)]
    def get_load_at_point_var(self, v_id: int, loc_id: int) -> cp_model.IntVar:
        return self.load_at_point[(v_id, loc_id)]
    def get_delivery_var(self, v_id: int, loc_id: int, brand_id: str) -> cp_model.IntVar:
        return self.delivered_vol[(v_id, loc_id, brand_id)]
    def get_pickup_var(self, v_id: int, loc_id: int, brand_id: str) -> cp_model.IntVar:
        return self.pickup_vol[(v_id, loc_id, brand_id)]
    def get_wh_active_var(self, wh_id: int) -> cp_model.BoolVar:
        return self.wh_active[wh_id]
    def get_wh_max_vol_var(self, wh_id: int) -> cp_model.IntVar:
        return self.wh_max_vol[wh_id]
    def get_wh_stock_change_per_visit_var(self, wh_id: int, brand_id: str, v_id: int) -> cp_model.IntVar:
        return self.wh_stock_change_per_visit[(wh_id, brand_id, v_id)]
    def get_wh_visit_interval_var(self, wh_id: int, v_id: int) -> cp_model.IntervalVar:
        return self.wh_visit_intervals[(wh_id, v_id)]
    def get_wh_visit_active_flag(self, wh_id: int, v_id: int) -> cp_model.BoolVar:
        return self.wh_visit_active_flags[(wh_id, v_id)]
    def get_vehicle_used_var(self, v_id: int) -> cp_model.BoolVar:
        return self.vehicle_used[v_id]
    def get_total_dist_var(self, v_id: int) -> cp_model.IntVar:
        return self.total_dist[v_id]
    def get_total_time_var(self, v_id: int) -> cp_model.IntVar:
        return self.total_time[v_id]
    def get_shift_start_var(self, v_id: int) -> cp_model.IntVar:
        return self.shift_start[v_id]
    def get_shift_end_var(self, v_id: int) -> cp_model.IntVar:
        return self.shift_end[v_id]