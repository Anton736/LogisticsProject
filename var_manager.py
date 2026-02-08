from ortools.sat.python import cp_model
from typing import Dict, Tuple, List, Optional
from entities import Scenario, Store, Warehouse


class VarManager:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario):
        self.model = model
        self.scenario = scenario

        # --- РЕЕСТРЫ ПЕРЕМЕННЫХ ---
        self.x = {}  # x_kij (движение: машина k из i в j)
        self.arrival_times = {}  # m_jt'k (время прибытия машины k в точку j)
        self.load_at_point = {}  # k_iq (загрузка ТС в ящиках при выезде из точки i)

        # Объемы (m_lt'vk0Wb и m_lt'vk1Wb)
        self.delivered_vol = {}  # m_lt'vk0 (сколько привезли бренда b в точку l машиной k)
        self.pickup_vol = {}  # m_lt'vk1 (сколько забрали бренда b из точки l машиной k)

        # Складские переменные
        self.wh_active = {}  # w_I (используется ли склад)
        self.wh_max_vol = {}  # w_iv (пиковый объем склада за все время)
        self.wh_stock_brand = {}  # w_it'b (остаток бренда b на складе после визита машины v)

        # Итоги по машинам для целевой функции
        self.vehicle_used = {}  # w_I для машин
        self.total_dist = {}  # k_is'' (общий путь)
        self.total_time = {}  # k_it'' (время смены: m_kfinish - m_kstart)
        self.shift_start = {}  # m_kstart (время начала работы водителя)
        self.shift_end = {}  # m_kfinish (время окончания работы водителя)

        # Запуск инициализации
        self._init_all_vars()

    def _init_all_vars(self):
        loc_ids = self.scenario.location_ids
        for v in self.scenario.vehicles:
            # Машины и параметры смен
            self.vehicle_used[v.id] = self.model.NewBoolVar(f'used_v{v.id}')
            self.total_dist[v.id] = self.model.NewIntVar(0, 500_000, f'dist_v{v.id}')
            self.total_time[v.id] = self.model.NewIntVar(0, 1440, f'total_t_v{v.id}')
            self.shift_start[v.id] = self.model.NewIntVar(0, 1440, f'start_v{v.id}')
            self.shift_end[v.id] = self.model.NewIntVar(0, 1440, f'end_v{v.id}')

            for loc in self.scenario.all_locations:
                # Временные метки и загрузка машины
                self.arrival_times[(v.id, loc.id)] = self.model.NewIntVar(0, 1440, f'arr_v{v.id}_l{loc.id}')
                self.load_at_point[(v.id, loc.id)] = self.model.NewIntVar(0, v.capacity, f'load_v{v.id}_l{loc.id}')

                # Детализация по брендам
                for b in self.scenario.brands:
                    self.delivered_vol[(v.id, loc.id, b.id)] = self.model.NewIntVar(0, v.capacity,
                                                                                    f'del_v{v.id}_l{loc.id}_b{b.id}')
                    self.pickup_vol[(v.id, loc.id, b.id)] = self.model.NewIntVar(0, v.capacity,
                                                                                 f'pick_v{v.id}_l{loc.id}_b{b.id}')

                # Маршрутные бинарные переменные
                for j_id in loc_ids:
                    if loc.id != j_id:
                        self.x[(v.id, loc.id, j_id)] = self.model.NewBoolVar(f'x_v{v.id}_{loc.id}_{j_id}')

        # Параметры складов
        for wh in self.scenario.warehouses:
            self.wh_active[wh.id] = self.model.NewBoolVar(f'wh_active_{wh.id}')
            self.wh_max_vol[wh.id] = self.model.NewIntVar(0, 1_000_000, f'wh_max_v{wh.id}')

            for b in self.scenario.brands:
                for v in self.scenario.vehicles:
                    # Динамический остаток (w_it'b)
                    self.wh_stock_brand[(wh.id, b.id, v.id)] = self.model.NewIntVar(0, 1_000_000,
                                                                                    f'stock_w{wh.id}_b{b.id}_v{v.id}')

    # --- Геттеры для СЛОЯ 3 ---

    def get_routing_var(self, v_id: int, i: int, j: int) -> Optional[cp_model.BoolVar]:
        return self.x.get((v_id, i, j))

    def get_arrival_var(self, v_id: int, loc_id: int) -> cp_model.IntVar:
        return self.arrival_times[(v_id, loc_id)]

    def get_delivery_var(self, v_id: int, loc_id: int, brand_id: str) -> cp_model.IntVar:
        return self.delivered_vol[(v_id, loc_id, brand_id)]

    def get_pickup_var(self, v_id: int, loc_id: int, brand_id: str) -> cp_model.IntVar:
        return self.pickup_vol[(v_id, loc_id, brand_id)]