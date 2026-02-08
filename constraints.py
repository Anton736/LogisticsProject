# START OF FILE constraints.py
from ortools.sat.python import cp_model
from typing import List, Tuple, Dict, Any

from entities import Scenario, Store, Warehouse, Location, Vehicle, Brand
from var_manager import VarManager
from RouterPruner import RoutePruner
from DemandStep import DemandManager


class ConstraintFactory:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario, var_manager: VarManager, demand_manager: DemandManager, pruner: RoutePruner):
        self.model = model
        self.scenario = scenario
        self.var_manager = var_manager
        self.demand_manager = demand_manager
        self.pruner = pruner
        self.location_by_id = {loc.id: loc for loc in scenario.all_locations}

    def add_all_constraints(self):
        print("Adding routing constraints...")
        self._add_routing_constraints()
        print("Adding time window and arrival constraints...")
        self._add_time_window_constraints()
        print("Adding load flow and capacity constraints...")
        self._add_load_flow_constraints()
        print("Adding demand satisfaction constraints...")
        self._add_demand_satisfaction_constraints()
        print("Adding warehouse activity and stock constraints (using Reservoir)...")
        self._add_warehouse_constraints()
        print("Adding vehicle activity linkage constraints...")
        self._add_vehicle_activity_linkage_constraints()
        print("All constraints added.")

    def _get_service_time_expr(self, v: Vehicle, loc: Location) -> cp_model.LinearExpr:
        total_vol_at_loc = self.model.NewIntVar(0, v.capacity * 2, f"total_vol_v{v.id}_l{loc.id}_service")
        self.model.Add(total_vol_at_loc == sum(self.var_manager.get_delivery_var(v.id, loc.id, b.id) for b in self.scenario.brands) +
                                           sum(self.var_manager.get_pickup_var(v.id, loc.id, b.id) for b in self.scenario.brands))

        if isinstance(loc, Store):
            if v.unloading_speed < 1: return self.model.NewConstant(0)
            return self.model.NewDiv(total_vol_at_loc, int(v.unloading_speed))
        elif isinstance(loc, Warehouse):
            if loc.handling_speed < 1: return self.model.NewConstant(0)
            return self.model.NewDiv(total_vol_at_loc, int(loc.handling_speed))
        return self.model.NewConstant(0)
    def _add_routing_constraints(self):
        """
        Реализует ограничения на маршруты:
        - Каждая активная машина должна начать и закончить маршрут на складе (depot).
        - Консервация потока (вход = выход для промежуточных точек).
        - Связь x_kij с общим пройденным расстоянием (total_dist).
        """
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            # 1. Начало и конец маршрута на складе (Depot)
            start_arcs = []
            end_arcs = []
            for wh in self.scenario.warehouses:
                for j in self.scenario.all_locations:
                    x_var = self.var_manager.get_routing_var(v.id, wh.id, j.id)
                    if x_var: start_arcs.append(x_var)
                for i in self.scenario.all_locations:
                    x_var = self.var_manager.get_routing_var(v.id, i.id, wh.id)
                    if x_var: end_arcs.append(x_var)

            self.model.Add(sum(start_arcs) == self.var_manager.get_vehicle_used_var(v.id))
            self.model.Add(sum(end_arcs) == self.var_manager.get_vehicle_used_var(v.id))

            # 2. Консервация потока для всех промежуточных точек (не складов)
            for loc in self.scenario.all_locations:
                if isinstance(loc, Warehouse): continue

                incoming_vars = []
                outgoing_vars = []
                for i_id, j_id in allowed_pairs:
                    x_var = self.var_manager.get_routing_var(v.id, i_id, j.id)
                    if x_var:
                        if j_id == loc.id: incoming_vars.append(x_var)
                        if i_id == loc.id: outgoing_vars.append(x_var)

                self.model.Add(sum(incoming_vars) == sum(outgoing_vars))
                self.model.Add(sum(incoming_vars) <= 1)

            # 3. Связь с общим пройденным расстоянием (k_is'')
            path_dist_terms = []
            for i_id, j_id in allowed_pairs:
                x_var = self.var_manager.get_routing_var(v.id, i_id, j.id)
                if x_var:
                    dist = self.scenario.network.distance_matrix[i_id][j.id]
                    path_dist_terms.append(x_var * int(dist * 100))

            total_dist_scaled = self.model.NewIntVar(0, 1_000_000 * 100, f'dist_scaled_v{v.id}')
            self.model.Add(total_dist_scaled == sum(path_dist_terms)).OnlyEnforceIf(
                self.var_manager.get_vehicle_used_var(v.id))

            self.model.Add(
                self.var_manager.get_total_dist_var(v.id) == self.model.NewDiv(total_dist_scaled, 100)).OnlyEnforceIf(
                self.var_manager.get_vehicle_used_var(v.id))
            self.model.Add(self.var_manager.get_total_dist_var(v.id) == 0).OnlyEnforceIf(
                self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))

    def _add_time_window_constraints(self):
        """
        Реализует ограничения на временные окна и расчет времени прибытия:
        - m_jt'k = max(m_it'k + m_itj + service_time, m_jt0).
        - Соблюдение временных окон магазинов (m_it0, m_it1).
        - Связь shift_start, shift_end, total_time с маршрутом.
        """
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            for loc in self.scenario.all_locations:
                self.model.Add(self.var_manager.get_arrival_var(v.id, loc.id) == 0).OnlyEnforceIf(
                    self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))

            for wh in self.scenario.warehouses:
                is_starting_wh = self.model.NewBoolVar(f"is_start_wh_v{v.id}_wh{wh.id}")
                outgoing_from_wh = [self.var_manager.get_routing_var(v.id, wh.id, j.id) for j in
                                    self.scenario.all_locations if self.var_manager.get_routing_var(v.id, wh.id, j.id)]
                if outgoing_from_wh:
                    self.model.AddBoolOr(outgoing_from_wh).OnlyEnforceIf(is_starting_wh)
                else:
                    self.model.Add(is_starting_wh == 0)

                self.model.Add(self.var_manager.get_arrival_var(v.id, wh.id) == self.var_manager.get_shift_start_var(
                    v.id)).OnlyEnforceIf(is_starting_wh)

            for i in self.scenario.all_locations:
                service_time_i = self._get_service_time_expr(v, i)
                for j in self.scenario.all_locations:
                    x_kij = self.var_manager.get_routing_var(v.id, i.id, j.id)
                    if not x_kij: continue

                    arrival_i = self.var_manager.get_arrival_var(v.id, i.id)
                    arrival_j = self.var_manager.get_arrival_var(v.id, j.id)
                    travel_t_ij = self.scenario.network.time_matrix[i.id][j.id]

                    self.model.Add(arrival_j >= arrival_i + service_time_i + travel_t_ij).OnlyEnforceIf(x_kij)

                    location_j = self.location_by_id[j.id]
                    min_arrival_j = location_j.time_start if isinstance(location_j, Store) else 0
                    max_arrival_j = location_j.time_end if isinstance(location_j, Store) else 1440

                    self.model.Add(arrival_j >= min_arrival_j).OnlyEnforceIf(x_kij)
                    self.model.Add(arrival_j <= max_arrival_j).OnlyEnforceIf(x_kij)

            if not self.scenario.all_locations:
                self.model.Add(self.var_manager.get_shift_start_var(v.id) == 0)
                self.model.Add(self.var_manager.get_shift_end_var(v.id) == 0)
                self.model.Add(self.var_manager.get_total_time_var(v.id) == 0)
                continue

            all_relevant_arrival_times = []
            all_relevant_departure_times = []

            for loc in self.scenario.all_locations:
                is_visited_v_loc = self.model.NewBoolVar(f"is_v{v.id}_visited_l{loc.id}_time_calc")
                incoming_or_outgoing_arcs = []
                for i_id, j_id in allowed_pairs:
                    x_var = self.var_manager.get_routing_var(v.id, i_id, j.id)
                    if x_var:
                        if i_id == loc.id: incoming_or_outgoing_arcs.append(x_var)
                        if j_id == loc.id: incoming_or_outgoing_arcs.append(x_var)

                if incoming_or_outgoing_arcs:
                    self.model.AddBoolOr(incoming_or_outgoing_arcs).OnlyEnforceIf(is_visited_v_loc)
                    self.model.Add(is_visited_v_loc == 0).OnlyEnforceIf(
                        self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))
                else:
                    self.model.Add(is_visited_v_loc == 0)

                arrival_time_if_visited = self.model.NewIntVar(0, 1440, f"arr_if_visited_v{v.id}_l{loc.id}")
                self.model.Add(arrival_time_if_visited == self.var_manager.get_arrival_var(v.id, loc.id)).OnlyEnforceIf(
                    is_visited_v_loc)
                self.model.Add(arrival_time_if_visited == 1440).OnlyEnforceIf(self.model.Not(is_visited_v_loc))
                all_relevant_arrival_times.append(arrival_time_if_visited)

                service_time_loc = self._get_service_time_expr(v, loc)
                departure_time_from_loc = self.model.NewIntVar(0, 1440 * 2, f"dep_v{v.id}_l{loc.id}")
                self.model.Add(
                    departure_time_from_loc == self.var_manager.get_arrival_var(v.id, loc.id) + service_time_loc)

                departure_time_if_visited = self.model.NewIntVar(0, 1440 * 2, f"dep_if_visited_v{v.id}_l{loc.id}")
                self.model.Add(departure_time_if_visited == departure_time_from_loc).OnlyEnforceIf(is_visited_v_loc)
                self.model.Add(departure_time_if_visited == 0).OnlyEnforceIf(self.model.Not(is_visited_v_loc))
                all_relevant_departure_times.append(departure_time_if_visited)

            min_arrival_time_expr = self.model.NewIntVar(0, 1440, f"min_arr_v{v.id}")
            self.model.AddMinEquality(min_arrival_time_expr, all_relevant_arrival_times)
            self.model.Add(self.var_manager.get_shift_start_var(v.id) == min_arrival_time_expr).OnlyEnforceIf(
                self.var_manager.get_vehicle_used_var(v.id))

            max_departure_time_expr = self.model.NewIntVar(0, 1440 * 2, f"max_dep_v{v.id}")
            self.model.AddMaxEquality(max_departure_time_expr, all_relevant_departure_times)
            self.model.Add(self.var_manager.get_shift_end_var(v.id) == max_departure_time_expr).OnlyEnforceIf(
                self.var_manager.get_vehicle_used_var(v.id))

            self.model.Add(self.var_manager.get_total_time_var(v.id) == self.var_manager.get_shift_end_var(
                v.id) - self.var_manager.get_shift_start_var(v.id)).OnlyEnforceIf(
                self.var_manager.get_vehicle_used_var(v.id))

            self.model.Add(self.var_manager.get_shift_start_var(v.id) == 0).OnlyEnforceIf(
                self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))
            self.model.Add(self.var_manager.get_shift_end_var(v.id) == 0).OnlyEnforceIf(
                self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))
            self.model.Add(self.var_manager.get_total_time_var(v.id) == 0).OnlyEnforceIf(
                self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))

    def _add_load_flow_constraints(self):
        """
        Реализует ограничения на поток груза и вместимость ТС:
        - k_iq (load_at_point) <= k_iv (vehicle capacity). (Ограничено при создании переменных)
        - k_jq = k_iq - m_jt'vk0 + m_jt'vk1 (поток груза).
        - m_jt'vk0 <= load_arriving_j (привезенный объем не может превышать прибывший).
        - Начальная и конечная загрузка ТС (стартует пустым, возвращается пустым).
        """
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            for wh in self.scenario.warehouses:
                is_starting_wh = self.model.NewBoolVar(f"is_start_wh_load_v{v.id}_wh{wh.id}")
                outgoing_from_wh = [self.var_manager.get_routing_var(v.id, wh.id, j.id) for j in
                                    self.scenario.all_locations if self.var_manager.get_routing_var(v.id, wh.id, j.id)]
                if outgoing_from_wh:
                    self.model.AddBoolOr(outgoing_from_wh).OnlyEnforceIf(is_starting_wh)
                else:
                    self.model.Add(is_starting_wh == 0)

                self.model.Add(self.var_manager.get_load_arriving_var(v.id, wh.id) == 0).OnlyEnforceIf(is_starting_wh)
                total_pickup_at_wh = sum(
                    self.var_manager.get_pickup_var(v.id, wh.id, b.id) for b in self.scenario.brands)
                self.model.Add(self.var_manager.get_load_at_point_var(v.id, wh.id) == total_pickup_at_wh).OnlyEnforceIf(
                    is_starting_wh)

            for i in self.scenario.all_locations:
                for j in self.scenario.all_locations:
                    x_kij = self.var_manager.get_routing_var(v.id, i.id, j.id)
                    if not x_kij: continue

                    self.model.Add(
                        self.var_manager.get_load_arriving_var(v.id, j.id) == self.var_manager.get_load_at_point_var(
                            v.id, i.id)).OnlyEnforceIf(x_kij)

            for loc in self.scenario.all_locations:
                total_delivered_at_loc = sum(
                    self.var_manager.get_delivery_var(v.id, loc.id, b.id) for b in self.scenario.brands)
                total_pickup_at_loc = sum(
                    self.var_manager.get_pickup_var(v.id, loc.id, b.id) for b in self.scenario.brands)

                self.model.Add(total_delivered_at_loc == 0).OnlyEnforceIf(
                    self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))
                self.model.Add(total_pickup_at_loc == 0).OnlyEnforceIf(
                    self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))
                self.model.Add(self.var_manager.get_load_arriving_var(v.id, loc.id) == 0).OnlyEnforceIf(
                    self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))
                self.model.Add(self.var_manager.get_load_at_point_var(v.id, loc.id) == 0).OnlyEnforceIf(
                    self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))

                is_loc_visited = self.model.NewBoolVar(f"is_v{v.id}_visited_l{loc.id}_load_flow")
                incoming_or_outgoing_arcs = []
                for i_id, j_id in allowed_pairs:
                    x_var = self.var_manager.get_routing_var(v.id, i_id, j.id)
                    if x_var:
                        if i_id == loc.id: incoming_or_outgoing_arcs.append(x_var)
                        if j_id == loc.id: incoming_or_outgoing_arcs.append(x_var)

                if incoming_or_outgoing_arcs:
                    self.model.AddBoolOr(incoming_or_outgoing_arcs).OnlyEnforceIf(is_loc_visited)
                    self.model.Add(is_loc_visited == 0).OnlyEnforceIf(
                        self.model.Not(self.var_manager.get_vehicle_used_var(v.id)))
                else:
                    self.model.Add(is_loc_visited == 0)

                self.model.Add(self.var_manager.get_load_at_point_var(v.id, loc.id) == \
                               self.var_manager.get_load_arriving_var(v.id,
                                                                      loc.id) - total_delivered_at_loc + total_pickup_at_loc).OnlyEnforceIf(
                    is_loc_visited)

                self.model.Add(
                    total_delivered_at_loc <= self.var_manager.get_load_arriving_var(v.id, loc.id)).OnlyEnforceIf(
                    is_loc_visited)
                self.model.Add(self.var_manager.get_load_at_point_var(v.id, loc.id) <= v.capacity).OnlyEnforceIf(
                    is_loc_visited)

            for wh in self.scenario.warehouses:
                is_ending_wh = self.model.NewBoolVar(f"is_end_wh_load_v{v.id}_wh{wh.id}")
                incoming_to_wh = [self.var_manager.get_routing_var(v.id, i.id, wh.id) for i in
                                  self.scenario.all_locations if self.var_manager.get_routing_var(v.id, i.id, wh.id)]
                if incoming_to_wh:
                    self.model.AddBoolOr(incoming_to_wh).OnlyEnforceIf(is_ending_wh)
                else:
                    self.model.Add(is_ending_wh == 0)

                self.model.Add(self.var_manager.get_load_at_point_var(v.id, wh.id) == 0).OnlyEnforceIf(is_ending_wh)

    def _add_demand_satisfaction_constraints(self):
        """
        Реализует ограничения на удовлетворение спроса магазинов:
        - sum(m_lt'vk0Wb - m_lt'vk1Wb, K and B) <= n_it'vb.
        - Спрос n_it'vb зависит от времени прибытия (DemandManager).
        """
        for store in self.scenario.stores:
            for brand in self.scenario.brands:
                # 1. Суммарный чистый объем доставленного бренда B в магазин L всеми машинами
                total_net_delivered_brand_to_store_by_all_vehicles = self.model.NewIntVar(
                    -sum(v.capacity for v in self.scenario.vehicles),  # Макс. возможный объем забора
                    sum(v.capacity for v in self.scenario.vehicles),  # Макс. возможный объем доставки
                    f"total_net_del_s{store.id}_b{brand.id}"
                )
                all_vehicle_net_delivery_terms = []
                for v in self.scenario.vehicles:
                    all_vehicle_net_delivery_terms.append(self.var_manager.get_delivery_var(v.id, store.id, brand.id) -
                                                          self.var_manager.get_pickup_var(v.id, store.id, brand.id))

                self.model.Add(
                    total_net_delivered_brand_to_store_by_all_vehicles == sum(all_vehicle_net_delivery_terms))

                # 2. Определение "репрезентативного" времени прибытия для магазина и бренда
                # Это будет минимальное время прибытия любой машины, которая доставляет данный бренд в данный магазин.
                earliest_arrival_for_brand_at_store = self.model.NewIntVar(0, 1440,
                                                                           f"earliest_arr_s{store.id}_b{brand.id}")

                relevant_arrival_times_for_min = []
                # Вот правильное имя переменной, которую мы будем использовать ниже
                is_any_delivery_for_brand_to_store = self.model.NewBoolVar(f"is_any_del_s{store.id}_b{brand.id}")
                is_delivering_flags = []

                for v in self.scenario.vehicles:
                    is_vehicle_delivering_brand_to_store = self.model.NewBoolVar(
                        f"is_del_v{v.id}_s{store.id}_b{brand.id}")
                    # Если машина доставляет (>0), то флаг true
                    self.model.Add(self.var_manager.get_delivery_var(v.id, store.id, brand.id) > 0).OnlyEnforceIf(
                        is_vehicle_delivering_brand_to_store)
                    self.model.Add(self.var_manager.get_delivery_var(v.id, store.id, brand.id) == 0).OnlyEnforceIf(
                        self.model.Not(is_vehicle_delivering_brand_to_store))

                    is_delivering_flags.append(is_vehicle_delivering_brand_to_store)

                    # Используем arrival_time если машина доставляет, иначе - большой фиктивный интервал (1440)
                    arrival_if_delivering = self.model.NewIntVar(0, 1440, f"arr_if_del_v{v.id}_s{store.id}_b{brand.id}")
                    self.model.Add(
                        arrival_if_delivering == self.var_manager.get_arrival_var(v.id, store.id)).OnlyEnforceIf(
                        is_vehicle_delivering_brand_to_store)
                    self.model.Add(arrival_if_delivering == 1440).OnlyEnforceIf(
                        self.model.Not(is_vehicle_delivering_brand_to_store))
                    relevant_arrival_times_for_min.append(arrival_if_delivering)

                # Если вообще есть поставки этого бренда в этот магазин, определяем минимальное время
                if is_delivering_flags:
                    self.model.AddBoolOr(is_delivering_flags).OnlyEnforceIf(is_any_delivery_for_brand_to_store)
                    self.model.Add(sum(is_delivering_flags) == 0).OnlyEnforceIf(
                        self.model.Not(is_any_delivery_for_brand_to_store))

                    self.model.AddMinEquality(earliest_arrival_for_brand_at_store,
                                              relevant_arrival_times_for_min).OnlyEnforceIf(
                        is_any_delivery_for_brand_to_store)
                    # Если поставок нет, то earliest_arrival_for_brand_at_store не имеет смысла.
                    # Установим его в 0 (или другое нейтральное значение, которое не будет активировать demand_manager)
                    self.model.Add(earliest_arrival_for_brand_at_store == 0).OnlyEnforceIf(
                        self.model.Not(is_any_delivery_for_brand_to_store))
                else:
                    # Если машин нет, то и поставок нет
                    self.model.Add(is_any_delivery_for_brand_to_store == 0)
                    self.model.Add(earliest_arrival_for_brand_at_store == 0)

                # 3. Получение максимально допустимого спроса n_it'vb от DemandManager
                # DemandManager.get_max_demand_at_time ожидает базовый спрос.
                # Из entities.Store.demands мы можем получить базовый спрос для слота 0 (или взять средний/первый)
                base_demand_for_brand = store.demands.get(brand.id, {}).get(0,
                                                                            0)  # Предполагаем, что 0-й слот - это базовый спрос
                max_allowed_demand_var = self.demand_manager.get_max_demand_at_time(
                    self.model, earliest_arrival_for_brand_at_store, base_demand_for_brand
                )

                # 4. Ограничение: чистый доставленный объем <= максимально допустимый спрос
                # Вот исправленная строка, которая использует переменную, а не несуществующий метод.
                self.model.Add(
                    total_net_delivered_brand_to_store_by_all_vehicles <= max_allowed_demand_var).OnlyEnforceIf(
                    is_any_delivery_for_brand_to_store)

                # Если поставок нет, то net_delivered должен быть <= 0 (например, может быть только забор)
                # Это ограничение уже покрывается предыдущим, так как если is_any_delivery == false,
                # то earliest_arrival_for_brand_at_store=0, что дает базовый max_demand.
                # Но для ясности можно добавить:
                self.model.Add(total_net_delivered_brand_to_store_by_all_vehicles <= 0).OnlyEnforceIf(
                    self.model.Not(is_any_delivery_for_brand_to_store))

    def _add_warehouse_constraints(self):
        """
        Реализует ограничения для складов с использованием ReservoirConstraint.
        """
        for wh in self.scenario.warehouses:
            # 1. Активация склада и интервалов визитов
            all_visits_to_wh = []
            for v in self.scenario.vehicles:
                # Флаг wh_visit_active_flags[(wh.id, v.id)] активен, если машина v посещает склад wh.
                is_v_visits_wh = self.var_manager.get_wh_visit_active_flag(wh.id, v.id)
                all_visits_to_wh.append(is_v_visits_wh)

                arcs_in_out = [var for (vid, i, j), var in self.var_manager.x.items() if
                               vid == v.id and (i == wh.id or j == wh.id)]
                if arcs_in_out:
                    self.model.Add(sum(arcs_in_out) > 0).OnlyEnforceIf(is_v_visits_wh)
                    self.model.Add(sum(arcs_in_out) == 0).OnlyEnforceIf(self.model.Not(is_v_visits_wh))
                else:
                    self.model.Add(is_v_visits_wh == 0)

                # Связываем параметры интервала (Start, Duration) с переменными модели
                interval = self.var_manager.get_wh_visit_interval_var(wh.id, v.id)
                arrival_time = self.var_manager.get_arrival_var(v.id, wh.id)
                service_time = self._get_service_time_expr(v, wh)

                # Эти ограничения должны быть условными, так как они имеют смысл только если визит состоялся.
                self.model.Add(interval.StartExpr() == arrival_time).OnlyEnforceIf(is_v_visits_wh)
                self.model.Add(interval.DurationExpr() == service_time).OnlyEnforceIf(is_v_visits_wh)

            # Связываем активацию склада (wh_active) с фактом визитов
            wh_active_var = self.var_manager.get_wh_active_var(wh.id)
            if all_visits_to_wh:
                self.model.Add(sum(all_visits_to_wh) > 0).OnlyEnforceIf(wh_active_var)
                self.model.Add(sum(all_visits_to_wh) == 0).OnlyEnforceIf(self.model.Not(wh_active_var))
            else:
                self.model.Add(wh_active_var == 0)

            # 2. Создание ReservoirConstraint для каждого бренда
            for b in self.scenario.brands:
                intervals_for_brand = [self.var_manager.get_wh_visit_interval_var(wh.id, v.id) for v in
                                       self.scenario.vehicles]
                demands_for_brand = []

                for v in self.scenario.vehicles:
                    stock_change = self.var_manager.get_wh_stock_change_per_visit_var(wh.id, b.id, v.id)
                    delivery_vol = self.var_manager.get_delivery_var(v.id, wh.id, b.id)
                    pickup_vol = self.var_manager.get_pickup_var(v.id, wh.id, b.id)
                    self.model.Add(stock_change == delivery_vol - pickup_vol)
                    demands_for_brand.append(stock_change)

                # ИСПРАВЛЕННАЯ СТРОКА с современным API
                initial_stock = wh.initial_stock.get(b.id, 0)
                max_capacity = 10_000_000
                self.model.AddReservoirConstraintWithConstantDelta(
                    intervals=intervals_for_brand,
                    demands=demands_for_brand,
                    min_level=0,
                    max_level=max_capacity
                ).AddConstantTermToLevel(initial_stock)

            # 3. Расчет wh_max_vol (w_iv) как общего потока для стоимости
            total_flow = self.model.NewIntVar(0, 10_000_000, f"total_flow_w{wh.id}")
            total_delivered_to_wh = sum(
                self.var_manager.get_delivery_var(v.id, wh.id, b.id) for v in self.scenario.vehicles for b in
                self.scenario.brands)
            total_pickup_from_wh = sum(
                self.var_manager.get_pickup_var(v.id, wh.id, b.id) for v in self.scenario.vehicles for b in
                self.scenario.brands)
            self.model.Add(total_flow == total_delivered_to_wh + total_pickup_from_wh)

            self.model.Add(self.var_manager.get_wh_max_vol_var(wh.id) == total_flow).OnlyEnforceIf(wh_active_var)
            self.model.Add(self.var_manager.get_wh_max_vol_var(wh.id) == 0).OnlyEnforceIf(self.model.Not(wh_active_var))

            # 4. Ограничение на забор брендов, не производимых складом
            for b in self.scenario.brands:
                if b.id not in wh.produced_brands:
                    for v in self.scenario.vehicles:
                        self.model.Add(self.var_manager.get_pickup_var(v.id, wh.id, b.id) == 0)

    def _add_vehicle_activity_linkage_constraints(self):
        for v in self.scenario.vehicles:
            is_used = self.var_manager.get_vehicle_used_var(v.id)
            all_arcs = [var for (vid, i, j), var in self.var_manager.x.items() if vid == v.id]
            if all_arcs:
                self.model.Add(sum(all_arcs) > 0).OnlyEnforceIf(is_used)
                self.model.Add(sum(all_arcs) == 0).OnlyEnforceIf(self.model.Not(is_used))
            else:
                self.model.Add(is_used == 0)