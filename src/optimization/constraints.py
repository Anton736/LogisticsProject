# START OF FILE constraints.py
from ortools.sat.python import cp_model


from src.core.entities import Scenario, Store, Warehouse, Location, Vehicle
from src.optimization.var_manager import VarManager
from src.optimization.pruner import RoutePruner
from src.models.demand import DemandManager
from src.core.enums import WarehouseCostMode

class ConstraintFactory:
    def __init__(self, model: cp_model.CpModel, scenario: Scenario, var_manager: VarManager, demand_manager: DemandManager, pruner: RoutePruner, warehouse_cost_mode: WarehouseCostMode = WarehouseCostMode.PEAK_INPUT):
        self.model = model
        self.scenario = scenario
        self.var_manager = var_manager
        self.demand_manager = demand_manager
        self.pruner = pruner
        self.location_by_id = {loc.id: loc for loc in scenario.all_locations}
        self.warehouse_cost_mode = warehouse_cost_mode

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
        total_vol_at_loc = self.model.new_int_var(0, v.capacity * 2, f"total_vol_v{v.id}_l{loc.id}_service")
        self.model.add(total_vol_at_loc == sum(
            self.var_manager.get_delivery_var(v.id, loc.id, b.id) for b in self.scenario.brands) +
                       sum(self.var_manager.get_pickup_var(v.id, loc.id, b.id) for b in self.scenario.brands))

        if isinstance(loc, Store):
            if v.unloading_speed < 1e-6: return self.model.new_constant(0)
            #div_result = self.model.new_int_var(0, 1440, f"service_time_st_{loc.id}_v{v.id}")
            #self.model.add_division_equality(div_result, total_vol_at_loc, int(v.unloading_speed))
            #return div_result
            return self.model.new_constant(int(loc.service_time))

        elif isinstance(loc, Warehouse):
            if loc.handling_speed < 1e-6: return self.model.new_constant(0)
            div_result = self.model.new_int_var(0, 1440, f"service_time_wh_{loc.id}_v{v.id}")
            self.model.add_division_equality(div_result, total_vol_at_loc, int(loc.handling_speed))
            return div_result

        return self.model.new_constant(0)

    def _add_routing_constraints(self):
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            is_used = self.var_manager.get_vehicle_used_var(v.id)

            # 0. Запрет петель (машина не едет сама к себе)
            for loc in self.scenario.all_locations:
                x_var = self.var_manager.get_routing_var(v.id, loc.id, loc.id)
                if x_var is not None:
                    self.model.add(x_var == 0)

            # 1. Начало маршрута строго на складе (Depot)
            start_arcs = []
            for wh in self.scenario.warehouses:
                for j in self.scenario.all_locations:
                    x_var = self.var_manager.get_routing_var(v.id, wh.id, j.id)
                    if x_var is not None: start_arcs.append(x_var)

            self.model.add(sum(start_arcs) == is_used)

            # 2. Консервация потока для OPEN VRP (маршрут может закончиться где угодно)
            end_flags = []

            for loc in self.scenario.all_locations:
                incoming_vars = []
                outgoing_vars = []
                for i_id, j_id in allowed_pairs:
                    x_var = self.var_manager.get_routing_var(v.id, i_id, j_id)
                    if x_var is not None:
                        if j_id == loc.id: incoming_vars.append(x_var)
                        if i_id == loc.id: outgoing_vars.append(x_var)

                # В любую точку можно въехать максимум 1 раз
                self.model.add(sum(incoming_vars) <= 1)

                if isinstance(loc, Store):
                    # Для магазинов: Вход - Выход = Флаг остановки (is_end_here)
                    # Если машина зашла (1) и не вышла (0) -> is_end_here = 1 (Конец смены)
                    is_end_here = self.model.new_bool_var(f"end_loc_v{v.id}_l{loc.id}")
                    self.model.add(sum(incoming_vars) - sum(outgoing_vars) == is_end_here)
                    end_flags.append(is_end_here)
                else:
                    # Для складов: машина тоже может вернуться на склад в конце смены
                    is_end_wh = self.model.new_bool_var(f"end_wh_v{v.id}_w{loc.id}")
                    self.model.add(sum(incoming_vars) == is_end_wh)
                    end_flags.append(is_end_wh)

            # ГЛАВНОЕ ПРАВИЛО: Каждая активная машина должна сделать строго 1 конечную остановку
            self.model.add(sum(end_flags) == is_used)

            # 3. Связь с общим пройденным расстоянием
            path_dist_terms = []
            for i_id, j_id in allowed_pairs:
                x_var = self.var_manager.get_routing_var(v.id, i_id, j_id)
                if x_var is not None:
                    dist = self.scenario.network.distance_matrix[i_id][j_id]
                    path_dist_terms.append(x_var * int(dist * 100))

            total_dist_scaled = self.model.new_int_var(0, 1_000_000 * 100, f'dist_scaled_v{v.id}')
            self.model.add(total_dist_scaled == sum(path_dist_terms)).OnlyEnforceIf(is_used)
            self.model.add(total_dist_scaled == 0).OnlyEnforceIf(is_used.Not())

            self.model.add_division_equality(
                self.var_manager.get_total_dist_var(v.id),
                total_dist_scaled,
                100
            )

    def _add_load_flow_constraints(self):
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            for wh in self.scenario.warehouses:
                is_starting_wh = self.model.new_bool_var(f"is_start_wh_load_v{v.id}_wh{wh.id}")
                outgoing_from_wh = [self.var_manager.get_routing_var(v.id, wh.id, j.id) for j in
                                    self.scenario.all_locations if
                                    self.var_manager.get_routing_var(v.id, wh.id, j.id) is not None]
                if outgoing_from_wh:
                    self.model.add_bool_or(outgoing_from_wh).OnlyEnforceIf(is_starting_wh)
                else:
                    self.model.add(is_starting_wh == 0)

                self.model.add(self.var_manager.get_load_arriving_var(v.id, wh.id) == 0).OnlyEnforceIf(is_starting_wh)
                total_pickup_at_wh = sum(
                    self.var_manager.get_pickup_var(v.id, wh.id, b.id) for b in self.scenario.brands)
                self.model.add(self.var_manager.get_load_at_point_var(v.id, wh.id) == total_pickup_at_wh).OnlyEnforceIf(
                    is_starting_wh)

            for i in self.scenario.all_locations:
                for j in self.scenario.all_locations:
                    x_kij = self.var_manager.get_routing_var(v.id, i.id, j.id)
                    if x_kij is None: continue

                    self.model.add(
                        self.var_manager.get_load_arriving_var(v.id, j.id) == self.var_manager.get_load_at_point_var(
                            v.id, i.id)).OnlyEnforceIf(x_kij)

            for loc in self.scenario.all_locations:
                total_delivered_at_loc = sum(
                    self.var_manager.get_delivery_var(v.id, loc.id, b.id) for b in self.scenario.brands)
                total_pickup_at_loc = sum(
                    self.var_manager.get_pickup_var(v.id, loc.id, b.id) for b in self.scenario.brands)

                self.model.add(total_delivered_at_loc == 0).OnlyEnforceIf(
                    self.var_manager.get_vehicle_used_var(v.id).Not())
                self.model.add(total_pickup_at_loc == 0).OnlyEnforceIf(
                    self.var_manager.get_vehicle_used_var(v.id).Not())
                self.model.add(self.var_manager.get_load_arriving_var(v.id, loc.id) == 0).OnlyEnforceIf(
                    self.var_manager.get_vehicle_used_var(v.id).Not())
                self.model.add(self.var_manager.get_load_at_point_var(v.id, loc.id) == 0).OnlyEnforceIf(
                    self.var_manager.get_vehicle_used_var(v.id).Not())

                is_loc_visited = self.model.new_bool_var(f"is_v{v.id}_visited_l{loc.id}_load_flow")
                incoming_or_outgoing_arcs = []
                for i_id, j_id in allowed_pairs:
                    x_var = self.var_manager.get_routing_var(v.id, i_id, j_id)
                    if x_var is not None:
                        if i_id == loc.id: incoming_or_outgoing_arcs.append(x_var)
                        if j_id == loc.id: incoming_or_outgoing_arcs.append(x_var)

                if incoming_or_outgoing_arcs:
                    self.model.add_bool_or(incoming_or_outgoing_arcs).OnlyEnforceIf(is_loc_visited)
                    self.model.add(is_loc_visited == 0).OnlyEnforceIf(
                        self.var_manager.get_vehicle_used_var(v.id).Not())
                else:
                    self.model.add(is_loc_visited == 0)

                self.model.add(self.var_manager.get_load_at_point_var(v.id, loc.id) == \
                               self.var_manager.get_load_arriving_var(v.id,
                                                                      loc.id) - total_delivered_at_loc + total_pickup_at_loc).OnlyEnforceIf(
                    is_loc_visited)

                self.model.add(
                    total_delivered_at_loc <= self.var_manager.get_load_arriving_var(v.id, loc.id)).OnlyEnforceIf(
                    is_loc_visited)
                self.model.add(self.var_manager.get_load_at_point_var(v.id, loc.id) <= v.capacity).OnlyEnforceIf(
                    is_loc_visited)

            for wh in self.scenario.warehouses:
                is_ending_wh = self.model.new_bool_var(f"is_end_wh_load_v{v.id}_wh{wh.id}")
                incoming_to_wh = [self.var_manager.get_routing_var(v.id, i.id, wh.id) for i in
                                  self.scenario.all_locations if
                                  self.var_manager.get_routing_var(v.id, i.id, wh.id) is not None]
                if incoming_to_wh:
                    self.model.add_bool_or(incoming_to_wh).OnlyEnforceIf(is_ending_wh)
                else:
                    self.model.add(is_ending_wh == 0)

                self.model.add(self.var_manager.get_load_at_point_var(v.id, wh.id) == 0).OnlyEnforceIf(is_ending_wh)

    def _add_time_window_constraints(self):
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            is_used = self.var_manager.get_vehicle_used_var(v.id)

            for loc in self.scenario.all_locations:
                self.model.add(self.var_manager.get_arrival_var(v.id, loc.id) == 0).OnlyEnforceIf(is_used.Not())

            for wh in self.scenario.warehouses:
                is_starting_wh = self.model.new_bool_var(f"is_start_wh_v{v.id}_wh{wh.id}")
                outgoing = [self.var_manager.get_routing_var(v.id, wh.id, j.id)
                            for j in self.scenario.all_locations if
                            self.var_manager.get_routing_var(v.id, wh.id, j.id) is not None]

                if outgoing:
                    self.model.add_bool_or(outgoing).OnlyEnforceIf(is_starting_wh)
                    self.model.add(sum(outgoing) == 0).OnlyEnforceIf(is_starting_wh.Not())
                    self.model.add(
                        self.var_manager.get_arrival_var(v.id, wh.id) == self.var_manager.get_shift_start_var(
                            v.id)).OnlyEnforceIf(is_starting_wh)
                else:
                    self.model.add(is_starting_wh == 0)

            for i in self.scenario.all_locations:
                service_i = self._get_service_time_expr(v, i)
                for j in self.scenario.all_locations:
                    x_kij = self.var_manager.get_routing_var(v.id, i.id, j.id)
                    if x_kij is None: continue

                    arr_i = self.var_manager.get_arrival_var(v.id, i.id)
                    arr_j = self.var_manager.get_arrival_var(v.id, j.id)
                    travel_ij = self.scenario.network.time_matrix[i.id][j.id]

                    self.model.add(arr_j >= arr_i + service_i + travel_ij).OnlyEnforceIf(x_kij)

                    loc_j = self.location_by_id[j.id]
                    if isinstance(loc_j, Store):
                        self.model.add(arr_j >= loc_j.time_start).OnlyEnforceIf(x_kij)
                        self.model.add(arr_j <= loc_j.time_end).OnlyEnforceIf(x_kij)

            all_relevant_arrivals = []
            all_relevant_departures = []

            for loc in self.scenario.all_locations:
                is_visited = self.model.new_bool_var(f"v{v.id}_vis_{loc.id}")
                arcs = [self.var_manager.get_routing_var(v.id, arc_i, arc_j)
                        for arc_i, arc_j in allowed_pairs if (arc_i == loc.id or arc_j == loc.id)]

                if arcs:
                    self.model.add_bool_or(arcs).OnlyEnforceIf(is_visited)
                    self.model.add(sum(arcs) == 0).OnlyEnforceIf(is_visited.Not())
                else:
                    self.model.add(is_visited == 0)

                arr_val = self.model.new_int_var(0, 1440, f"arr_v{v.id}_{loc.id}")
                self.model.add(arr_val == self.var_manager.get_arrival_var(v.id, loc.id)).OnlyEnforceIf(is_visited)
                self.model.add(arr_val == 1440).OnlyEnforceIf(is_visited.Not())
                all_relevant_arrivals.append(arr_val)

                dep_val = self.model.new_int_var(0, 2880, f"dep_v{v.id}_{loc.id}")
                self.model.add(
                    dep_val == self.var_manager.get_arrival_var(v.id, loc.id) + self._get_service_time_expr(v,
                                                                                                            loc)).OnlyEnforceIf(
                    is_visited)
                self.model.add(dep_val == 0).OnlyEnforceIf(is_visited.Not())
                all_relevant_departures.append(dep_val)

            s_start = self.var_manager.get_shift_start_var(v.id)
            s_end = self.var_manager.get_shift_end_var(v.id)
            s_total = self.var_manager.get_total_time_var(v.id)

            if all_relevant_arrivals:
                self.model.add_min_equality(s_start, all_relevant_arrivals)
                self.model.add_max_equality(s_end, all_relevant_departures)
                self.model.add(s_total == s_end - s_start).OnlyEnforceIf(is_used)

            self.model.add(s_start == 0).OnlyEnforceIf(is_used.Not())
            self.model.add(s_end == 0).OnlyEnforceIf(is_used.Not())
            self.model.add(s_total == 0).OnlyEnforceIf(is_used.Not())

    def _add_demand_satisfaction_constraints(self):
        v_cap_sum = sum(v.capacity for v in self.scenario.vehicles)

        for store in self.scenario.stores:
            # ШАГ 1: Кэшируем временные сетки для всех машин в этой точке
            # Это экономит тысячи переменных, так как сетка не зависит от бренда
            veh_time_segments = {}
            for v in self.scenario.vehicles:
                arrival_var = self.var_manager.get_arrival_var(v.id, store.id)
                veh_time_segments[v.id] = self.demand_manager.get_segment_indicators(
                    self.model, arrival_var, store.id, v.id
                )

            for brand in self.scenario.brands:
                base_demand = store.demands.get(brand.id, {}).get(0, 0)
                if base_demand == 0:
                    continue

                # ШАГ 2: Считаем общую доставку бренда всеми машинами
                total_net_delivered = self.model.new_int_var(-v_cap_sum, v_cap_sum, f"net_s{store.id}_b{brand.id}")
                delivery_terms = []
                is_v_delivering_brand = {}  # Флаги: везет ли конкретная машина этот бренд

                for v in self.scenario.vehicles:
                    del_v = self.var_manager.get_delivery_var(v.id, store.id, brand.id)
                    pick_v = self.var_manager.get_pickup_var(v.id, store.id, brand.id)
                    delivery_terms.append(del_v - pick_v)

                    # Создаем флаг присутствия бренда в машине
                    is_del = self.model.new_bool_var(f"is_del_v{v.id}_s{store.id}_b{brand.id}")
                    self.model.add(del_v > 0).OnlyEnforceIf(is_del)
                    self.model.add(del_v == 0).OnlyEnforceIf(is_del.Not())
                    is_v_delivering_brand[v.id] = is_del

                self.model.add(total_net_delivered == sum(delivery_terms))

                # ШАГ 3: Определяем финальный лимит (Логика "самого раннего прибытия")
                # Мы идем по сегментам времени 0, 1, 2...
                # Лимит срабатывает по первому сегменту, в котором оказалась ХОТЯ БЫ ОДНА машина с этим брендом.

                final_limit = self.model.new_int_var(0, int(base_demand * 2), f"final_lim_s{store.id}_b{brand.id}")

                # Флаги для каскада: "был ли бренд доставлен в сегменте i или ранее?"
                any_delivery_until_seg = []

                for i in range(len(self.demand_manager.steps)):
                    # 3a. Есть ли хоть одна машина с брендом в ТЕКУЩЕМ сегменте i?
                    v_in_current_seg = []
                    for v in self.scenario.vehicles:
                        v_active_here = self.model.new_bool_var("")
                        # Машина v в сегменте i И она везет бренд
                        self.model.add_bool_and([veh_time_segments[v.id][i], is_v_delivering_brand[v.id]]).OnlyEnforceIf(
                            v_active_here)
                        self.model.add_bool_or(
                            [veh_time_segments[v.id][i].Not(), is_v_delivering_brand[v.id].Not()]).OnlyEnforceIf(
                            v_active_here.Not())
                        v_in_current_seg.append(v_active_here)

                    has_any_v_in_seg_i = self.model.new_bool_var(f"has_v_s{store.id}_b{brand.id}_seg{i}")
                    self.model.add_bool_or(v_in_current_seg).OnlyEnforceIf(has_any_v_in_seg_i)
                    self.model.add(sum(v_in_current_seg) == 0).OnlyEnforceIf(has_any_v_in_seg_i.Not())

                    # 3b. Этот сегмент является решающим (самым ранним), если:
                    # В нем кто-то есть И в предыдущих никого не было.
                    is_this_earliest = self.model.new_bool_var(f"is_earliest_s{store.id}_b{brand.id}_seg{i}")
                    if i == 0:
                        self.model.add(is_this_earliest == has_any_v_in_seg_i)
                    else:
                        # Условие: has_any_v_in_seg_i AND NOT (предыдущие any_delivery_until_seg)
                        prev_combined = self.model.new_bool_var("")
                        self.model.add_bool_or(any_delivery_until_seg).OnlyEnforceIf(prev_combined)
                        self.model.add(sum(any_delivery_until_seg) == 0).OnlyEnforceIf(prev_combined.Not())

                        self.model.add_bool_and([has_any_v_in_seg_i, prev_combined.Not()]).OnlyEnforceIf(is_this_earliest)
                        self.model.add_bool_or([has_any_v_in_seg_i.Not(), prev_combined]).OnlyEnforceIf(
                            is_this_earliest.Not())

                    any_delivery_until_seg.append(has_any_v_in_seg_i)

                    # 3c. Применяем лимит этого сегмента, если он признан самым ранним
                    calc_demand = (base_demand * self.demand_manager.steps[i].multiplier_x100) // 100
                    self.model.add(final_limit == calc_demand).OnlyEnforceIf(is_this_earliest)

                # Если поставок вообще нет (any_delivery_until_seg все False),
                # то total_net_delivered и так будет <= 0 из-за определений переменных.
                # Но для страховки: если хоть одна поставка была, применяем лимит.
                is_any_del = self.model.new_bool_var("")
                self.model.add_bool_or(any_delivery_until_seg).OnlyEnforceIf(is_any_del)
                self.model.add(sum(any_delivery_until_seg) == 0).OnlyEnforceIf(is_any_del.Not())

                self.model.add(total_net_delivered <= final_limit).OnlyEnforceIf(is_any_del)
                self.model.add(total_net_delivered <= 0).OnlyEnforceIf(is_any_del.Not())



    def _add_vehicle_activity_linkage_constraints(self):
        for v in self.scenario.vehicles:
            is_used = self.var_manager.get_vehicle_used_var(v.id)
            all_arcs = [var for (vid, i, j), var in self.var_manager.x.items() if vid == v.id]
            if all_arcs:
                self.model.add(sum(all_arcs) > 0).OnlyEnforceIf(is_used)
                self.model.add(sum(all_arcs) == 0).OnlyEnforceIf(is_used.Not())
            else:
                self.model.add(is_used == 0)



    def _add_warehouse_constraints(self):
        """
        Реализует ограничения для складов: индикатор использования и Reservoir.
        """
        for wh in self.scenario.warehouses:
            wh_active = self.var_manager.get_wh_active_var(wh.id)
            all_visits = []

            for v in self.scenario.vehicles:
                is_v_at_wh = self.var_manager.get_wh_visit_active_flag(wh.id, v.id)
                all_visits.append(is_v_at_wh)

                # Связь с дугами
                arcs = [var for (vid, arc_i, arc_j), var in self.var_manager.x.items()
                        if vid == v.id and (arc_i == wh.id or arc_j == wh.id)]
                if arcs:
                    self.model.add_bool_or(arcs).OnlyEnforceIf(is_v_at_wh)
                    self.model.add(sum(arcs) == 0).OnlyEnforceIf(is_v_at_wh.Not())
                else:
                    self.model.add(is_v_at_wh == 0)

                # Reservoir: Интервал визита
                interval = self.var_manager.get_wh_visit_interval_var(wh.id, v.id)
                arrival = self.var_manager.get_arrival_var(v.id, wh.id)
                service = self._get_service_time_expr(v, wh)

                self.model.add(interval.StartExpr() == arrival).OnlyEnforceIf(is_v_at_wh)
                self.model.add(interval.SizeExpr() == service).OnlyEnforceIf(is_v_at_wh)

            # Активность склада (wh_active == 1 <=> хотя бы один визит)
            if all_visits:
                self.model.add_bool_or(all_visits).OnlyEnforceIf(wh_active)
                self.model.add(sum(all_visits) == 0).OnlyEnforceIf(wh_active.Not())
            else:
                self.model.add(wh_active == 0)

            # Баланс Reservoir
                # Баланс Reservoir
                # Баланс Reservoir (Современный Python API)
                for b in self.scenario.brands:
                    times = []
                    demands = []  # Изменения уровня
                    actives = []  # Флаги активности (1 = событие происходит)

                    # 1. Добавляем начальный запас склада в момент времени 0
                    initial_stock = wh.initial_stock.get(b.id, 0)
                    if initial_stock > 0:
                        times.append(0)
                        demands.append(initial_stock)
                        actives.append(self.model.new_constant(1))  # Всегда происходит

                    # 2. Собираем события от визитов машин
                    for v_obj in self.scenario.vehicles:
                        change = self.var_manager.get_wh_stock_change_per_visit_var(wh.id, b.id, v_obj.id)
                        self.model.add(change == self.var_manager.get_delivery_var(v_obj.id, wh.id, b.id) -
                                       self.var_manager.get_pickup_var(v_obj.id, wh.id, b.id))

                        time_var = self.var_manager.get_arrival_var(v_obj.id, wh.id)
                        is_active = self.var_manager.get_wh_visit_active_flag(wh.id, v_obj.id)

                        times.append(time_var)
                        demands.append(change)
                        actives.append(is_active)

                    # 3. Создаем резервуар одной строкой, передавая собранные списки
                    if times:  # Если есть хоть какие-то события
                        self.model.add_reservoir_constraint_with_active(
                            times, demands, actives, 0, 10_000_000
                        )

            # Объем склада (w_iv)
            wh_max_vol = self.var_manager.get_wh_max_vol_var(wh.id)
            if self.warehouse_cost_mode == WarehouseCostMode.PEAK_INPUT:
                total_delivered = sum(self.var_manager.get_delivery_var(v_obj.id, wh.id, b_obj.id)
                                      for v_obj in self.scenario.vehicles for b_obj in self.scenario.brands)
                self.model.add(wh_max_vol == sum(wh.initial_stock.values()) + total_delivered).OnlyEnforceIf(wh_active)
            elif self.warehouse_cost_mode == WarehouseCostMode.EXACT_PEAK:
                pass  # wh_max_vol вычисляется в _add_exact_peak_warehouse_stock_constraints

            self.model.add(wh_max_vol == 0).OnlyEnforceIf(wh_active.Not())

            # Запрет на забор чужих брендов
            for b in self.scenario.brands:
                if b.id not in wh.produced_brands:
                    for v_obj in self.scenario.vehicles:
                        self.model.add(self.var_manager.get_pickup_var(v_obj.id, wh.id, b.id) == 0)

        # EXACT_PEAK требует отдельного прохода после формирования всех before-переменных
        if self.warehouse_cost_mode == WarehouseCostMode.EXACT_PEAK:
            print("  Computing exact peak warehouse volumes via temporal event-chain...")
            self._add_exact_peak_warehouse_stock_constraints()

    def _get_eligible_vehicles_for_wh(self, wh_id: str) -> list:
        """
        K_eff: возвращает только машины, у которых существует хотя бы одна
        разрешённая дуга, связанная со складом wh_id (т.е. машина физически
        может туда приехать или оттуда уехать).

        Это позволяет пропустить машины которые гарантированно не посетят склад
        (например, после обрезки дуг в RoutePruner), уменьшая размер EXACT_PEAK.
        """
        eligible = []
        for v in self.scenario.vehicles:
            has_arc = any(
                self.var_manager.get_routing_var(v.id, wh_id, loc.id) is not None
                or self.var_manager.get_routing_var(v.id, loc.id, wh_id) is not None
                for loc in self.scenario.all_locations
                if loc.id != wh_id
            )
            if has_arc:
                eligible.append(v)
        return eligible

    def _add_exact_peak_warehouse_stock_constraints(self) -> None:
        """
        EXACT_PEAK: точный расчёт пикового объёма склада (w_iv) через событийную
        временну́ю цепочку.

        НЕ заменяет ReservoirConstraint — тот по-прежнему гарантирует, что
        запас не уходит в минус. Этот метод только честно вычисляет wh_max_vol,
        чтобы стоимость аренды склада считалась корректно.

        Реализованные улучшения:
          1. Event-Chain по РЕАЛЬНОМУ времени приезда (не по ID машины).
          2. before[k1,k2] фиксирует временной порядок → Symmetry Breaking.
          3. Ghost Pass: если машина не посещала склад, before=0 → contrib=0.
          4. Pressure from Below: wh_max_vol >= stock_after каждого визита
             (только линейные >= без тяжёлого AddMaxEquality).

        Дополнительные оптимизации (vs предыдущей версии):
          5. Заводы (wh.is_factory=True) пропускаются — их w_iv не оплачивается.
          6. K_eff: используются только машины с хотя бы одной дугой к РЦ.
             После обрезки в RoutePruner это может существенно уменьшить K.

        Сложность по переменным (на 1 РЦ):
          before:       K_eff*(K_eff-1)
          net_del:      K_eff*B
          contrib:      K_eff*(K_eff-1)*B
          stock_bef/aft: 2*K_eff*B
          При K_eff=4, B=3: ~66 доп. переменных — очень быстро.
          При K_eff=10, B=3: ~330 доп. переменных — быстро.
          При K_eff>30: рекомендуется PEAK_INPUT.
        """
        from typing import Dict, List, Tuple as T

        for wh in self.scenario.warehouses:
            # Заводы: w_iv не оплачивается и не нуждается в точном расчёте пика
            if wh.is_factory:
                continue


            wh_max_vol = self.var_manager.get_wh_max_vol_var(wh.id)

            # K_eff: только машины с хотя бы одной дугой к этому РЦ
            vehicles = self._get_eligible_vehicles_for_wh(wh.id)
            if not vehicles:
                # Нет ни одной возможной дуги к этому РЦ — он никогда не будет посещён
                self.model.add(wh_max_vol == 0)
                continue

            k_total = len(self.scenario.vehicles)
            k_eff   = len(vehicles)
            if k_eff < k_total:
                print(f"  EXACT_PEAK: warehouse {wh.id} uses K_eff={k_eff} (of {k_total} total vehicles)")

            # ----------------------------------------------------------------
            # ШАГ 1 — before[k1_id, k2_id]: k1 приезжает на wh РАНЬШЕ k2
            # ----------------------------------------------------------------
            before: Dict[T[int, int], cp_model.BoolVarT] = {}
            for k1 in vehicles:
                for k2 in vehicles:
                    if k1.id == k2.id:
                        continue
                    before[(k1.id, k2.id)] = self.model.new_bool_var(
                        f"bef_w{wh.id}_{k1.id}_{k2.id}"
                    )

            for k1 in vehicles:
                for k2 in vehicles:
                    if k1.id >= k2.id:
                        continue  # каждую пару разбираем один раз

                    b12   = before[(k1.id, k2.id)]
                    b21   = before[(k2.id, k1.id)]
                    is_k1 = self.var_manager.get_wh_visit_active_flag(wh.id, k1.id)
                    is_k2 = self.var_manager.get_wh_visit_active_flag(wh.id, k2.id)
                    arr1  = self.var_manager.get_arrival_var(k1.id, wh.id)
                    arr2  = self.var_manager.get_arrival_var(k2.id, wh.id)

                    # Оба посещают → ровно один из них "раньше"
                    self.model.add(b12 + b21 == 1).OnlyEnforceIf([is_k1, is_k2])

                    # Временна́я привязка: b12=1 → arr1 <= arr2
                    self.model.add(arr1 <= arr2).OnlyEnforceIf([b12, is_k1, is_k2])
                    self.model.add(arr2 <= arr1).OnlyEnforceIf([b21, is_k1, is_k2])

                    # k1 не посещает → k1 не может быть "раньше" k2
                    self.model.add(b12 == 0).OnlyEnforceIf(is_k1.Not())
                    # k2 не посещает → k2 не может быть "раньше" k1
                    self.model.add(b21 == 0).OnlyEnforceIf(is_k2.Not())

            # ----------------------------------------------------------------
            # ШАГИ 2-3 — net_del, contrib, stock_before, stock_after (по брендам)
            # ----------------------------------------------------------------
            # Накапливаем stock_after[k] по всем брендам для шага 4.
            stock_after_per_vehicle: Dict[int, List[cp_model.IntVar]] = {
                k.id: [] for k in vehicles
            }

            for b in self.scenario.brands:
                initial_b = wh.initial_stock.get(b.id, 0)
                max_possible = sum(k.capacity for k in vehicles) + initial_b

                # --- net_del[k] = delivered[k,wh,b] - pickup[k,wh,b] ---
                net_del: Dict[int, cp_model.IntVar] = {}
                for k in vehicles:
                    nd = self.model.new_int_var(
                        -k.capacity, k.capacity,
                        f"nd_w{wh.id}_b{b.id}_k{k.id}"
                    )
                    self.model.add(
                        nd == self.var_manager.get_delivery_var(k.id, wh.id, b.id)
                           - self.var_manager.get_pickup_var(k.id, wh.id, b.id)
                    )
                    net_del[k.id] = nd

                # --- contrib[k_src, k_dst] = net_del[k_src] * before[k_src, k_dst] ---
                # Линеаризация через OnlyEnforceIf (без AddMultiplicationEquality).
                # Ghost Pass: если k_src не посещал склад, before=0 → contrib=0.
                contrib: Dict[T[int, int], cp_model.IntVar] = {}
                for k_src in vehicles:
                    for k_dst in vehicles:
                        if k_src.id == k_dst.id:
                            continue
                        c = self.model.new_int_var(
                            -k_src.capacity, k_src.capacity,
                            f"ctr_w{wh.id}_b{b.id}_{k_src.id}_{k_dst.id}"
                        )
                        bv = before[(k_src.id, k_dst.id)]
                        self.model.add(c == net_del[k_src.id]).OnlyEnforceIf(bv)
                        self.model.add(c == 0).OnlyEnforceIf(bv.Not())
                        contrib[(k_src.id, k_dst.id)] = c

                # --- stock_before[k], stock_after[k] ---
                for k in vehicles:
                    is_k = self.var_manager.get_wh_visit_active_flag(wh.id, k.id)

                    # stock_before_k = initial_b + Σ contrib[k', k]  (k' ≠ k)
                    # "Сколько товара бренда b на складе прямо перед приездом машины k"
                    sum_before_k = initial_b + sum(
                        contrib[(k2.id, k.id)]
                        for k2 in vehicles if k2.id != k.id
                    )
                    stock_bef = self.model.new_int_var(
                        0, max_possible, f"sbef_w{wh.id}_b{b.id}_k{k.id}"
                    )
                    self.model.add(stock_bef == sum_before_k).OnlyEnforceIf(is_k)
                    self.model.add(stock_bef == 0).OnlyEnforceIf(is_k.Not())

                    # stock_after_k = stock_before_k + net_del[k]
                    # "Сколько товара бренда b остаётся после обслуживания машины k"
                    stock_aft = self.model.new_int_var(
                        0, max_possible, f"saft_w{wh.id}_b{b.id}_k{k.id}"
                    )
                    self.model.add(
                        stock_aft == stock_bef + net_del[k.id]
                    ).OnlyEnforceIf(is_k)
                    self.model.add(stock_aft == 0).OnlyEnforceIf(is_k.Not())

                    stock_after_per_vehicle[k.id].append(stock_aft)

            # ----------------------------------------------------------------
            # ШАГ 4 — "Давление снизу": wh_max_vol >= суммарный объём после каждого визита
            # ----------------------------------------------------------------
            # Линейные >= вместо AddMaxEquality — солвер сам "сдавит" wh_max_vol
            # до наименьшего значения, покрывающего все пики.
            max_total_brands = sum(k.capacity for k in vehicles) * len(self.scenario.brands)
            for k in vehicles:
                if not stock_after_per_vehicle[k.id]:
                    continue

                is_k = self.var_manager.get_wh_visit_active_flag(wh.id, k.id)

                # Суммарный объём всех брендов на складе после визита машины k
                total_after_k = self.model.new_int_var(
                    0, max_total_brands,
                    f"tot_saft_w{wh.id}_k{k.id}"
                )
                self.model.add(
                    total_after_k == sum(stock_after_per_vehicle[k.id])
                ).OnlyEnforceIf(is_k)
                self.model.add(total_after_k == 0).OnlyEnforceIf(is_k.Not())

                # Давление снизу: wh_max_vol должен покрыть этот пик
                self.model.add(wh_max_vol >= total_after_k).OnlyEnforceIf(is_k)