from ortools.sat.python import cp_model
from typing import Dict, Tuple, List
from entities import Scenario, Vehicle, Location


class VarManager:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario):
        self.model = model
        self.scenario = scenario

        # Реестры переменных
        self.x = {}  # Маршруты: (k, i, j) -> BoolVar
        self.arrival_times = {}  # Время прибытия: (k, i) -> IntVar
        self.load_at_point = {}  # Загрузка ТС при выезде: (k, i) -> IntVar
        self.delivered_vol = {}  # Выгрузка: (k, store_id, brand_id) -> IntVar
        self.wh_active = {}  # Индикатор склада: wh_id -> BoolVar
        self.wh_max_vol = {}  # Пиковый объем склада: wh_id -> IntVar

        # Технические переменные для целевой функции
        self.vehicle_used = {}  # Использована ли машина k -> BoolVar
        self.total_dist = {}  # Общий путь машины k -> IntVar
        self.total_time = {}  # Общее время смены машины k -> IntVar

        # Инициализация
        self._init_routing_vars()
        self._init_load_and_delivery_vars()
        self._init_time_vars()
        self._init_warehouse_vars()

    def _init_routing_vars(self):
        """Создание x_kij и вспомогательных переменных активности ТС"""
        loc_ids = self.scenario.location_ids
        for v in self.scenario.vehicles:
            # Машина используется, если она выехала из любого склада
            self.vehicle_used[v.id] = self.model.NewBoolVar(f'used_v{v.id}')

            for i in loc_ids:
                for j in loc_ids:
                    if i != j:
                        self.x[(v.id, i, j)] = self.model.NewBoolVar(f'x_v{v.id}_i{i}_j{j}')

            # Переменные для итогов по машине (нужны для числителя формулы)
            self.total_dist[v.id] = self.model.NewIntVar(0, 100_000, f'dist_v{v.id}')
            self.total_time[v.id] = self.model.NewIntVar(0, 1440, f'time_v{v.id}')

    def _init_load_and_delivery_vars(self):
        """Создание переменных для k_iq и m_lt'vk0Wb"""
        for v in self.scenario.vehicles:
            for loc in self.scenario.all_locations:
                # k_iq: объем в ящиках в момент выезда из точки i
                self.load_at_point[(v.id, loc.id)] = self.model.NewIntVar(
                    0, v.capacity, f'load_v{v.id}_loc{loc.id}'
                )

                # Если точка — это магазин, создаем переменные доставки по брендам
                if isinstance(loc, Store):
                    for brand in self.scenario.brands:
                        self.delivered_vol[(v.id, loc.id, brand.id)] = self.model.NewIntVar(
                            0, v.capacity, f'deliv_v{v.id}_s{loc.id}_b{brand.id}'
                        )

    def _init_time_vars(self):
        """Создание m_it'k (время прибытия)"""
        for v in self.scenario.vehicles:
            for loc in self.scenario.all_locations:
                # Время в минутах (0-1440)
                self.arrival_times[(v.id, loc.id)] = self.model.NewIntVar(
                    0, 1440, f'arr_v{v.id}_loc{loc.id}'
                )

    def _init_warehouse_vars(self):
        """Создание w_I и w_iv"""
        for wh in self.scenario.warehouses:
            self.wh_active[wh.id] = self.model.NewBoolVar(f'wh_active_{wh.id}')
            self.wh_max_vol[wh.id] = self.model.NewIntVar(0, 1_000_000, f'wh_vol_{wh.id}')

    # --- Геттеры для соблюдения инкапсуляции ---

    def get_routing_var(self, v_id: int, i: int, j: int):
        return self.x.get((v_id, i, j))

    def get_arrival_var(self, v_id: int, loc_id: int):
        return self.arrival_times.get((v_id, loc_id))

    def get_delivery_vars_for_store(self, store_id: int):
        """Возвращает все переменные доставки во все машины для конкретного магазина"""
        return [self.delivered_vol[k, s, b]
                for (k, s, b) in self.delivered_vol
                if s == store_id]
