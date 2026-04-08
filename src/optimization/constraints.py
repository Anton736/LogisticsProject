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
        print("Начинаем пошаговое добавление ограничений...")

        # 1. БАЗА: Роутинг (Машины могут ездить между точками)
        self._add_routing_constraints()

        # 2. СВЯЗКА: Машина используется, только если посетила магазин
        self._add_vehicle_activity_linkage_constraints()

        # --- НИЖЕ СТОЯТ "ТЯЖЕЛЫЕ" ОГРАНИЧЕНИЯ. ВКЛЮЧАЙ ИХ ПО ОДНОМУ ---

        # 3. ВРЕМЯ: Машины должны соблюдать окна и время пути
        self._add_time_window_constraints()

        # 4. ГРУЗ: Вместимость машин (ящики/штуки) и потоки
        self._add_load_flow_constraints()

        # 5. СПРОС: Нужно заезжать в магазины и отдавать товар
        self._add_demand_satisfaction_constraints()

        # 6. СКЛАДЫ: Reservoir и запасы на заводе
        # self._add_warehouse_constraints()

        # 7. ОПТИМИЗАЦИЯ: Симметрия (ускоряет, но не обязательна)
        #self._add_symmetry_breaking_constraints()

        print("Все активные ограничения добавлены.")

    def _add_symmetry_breaking_constraints(self):
        """
        Если машины идентичны, заставляем солвер использовать их по порядку.
        Машина k+1 может быть активна только если активна машина k.
        """
        vehicles = self.scenario.vehicles
        for i in range(len(vehicles) - 1):
            v_curr = vehicles[i]
            v_next = vehicles[i + 1]

            # Если у машин одинаковые характеристики (цена, вместимость)
            if (v_curr.capacity == v_next.capacity and
                    v_curr.cost_km == v_next.cost_km):
                used_curr = self.var_manager.get_vehicle_used_var(v_curr.id)
                used_next = self.var_manager.get_vehicle_used_var(v_next.id)

                # Запрещаем использовать следующую без предыдущей
                self.model.add(used_curr >= used_next)

    def _get_service_time_expr(self, v: Vehicle, loc: Location) -> cp_model.LinearExpr:
        # --- ФИКС БАГА 3 (Двойной вызов): используем кэш ---
        key = (v.id, loc.id)
        if key in self.var_manager.service_time_cache:
            return self.var_manager.service_time_cache[key]

        max_units = v.capacity * self.scenario.units_per_crate * 2
        total_vol_at_loc = self.model.new_int_var(0, max_units, f"total_vol_v{v.id}_l{loc.id}_service")

        self.model.add(total_vol_at_loc == sum(
            self.var_manager.get_delivery_var(v.id, loc.id, b.id) for b in self.scenario.brands) +
                       sum(self.var_manager.get_pickup_var(v.id, loc.id, b.id) for b in self.scenario.brands))

        if isinstance(loc, Store):
            expr = self.model.new_constant(int(loc.service_time))
        elif isinstance(loc, Warehouse):
            flow_vars = []
            for b in self.scenario.brands:
                d_v = self.var_manager.get_delivery_var(v.id, loc.id, b.id)
                p_v = self.var_manager.get_pickup_var(v.id, loc.id, b.id)
                if d_v is not None: flow_vars.append(d_v)
                if p_v is not None: flow_vars.append(p_v)

            if not flow_vars or loc.handling_speed < 1e-6:
                expr = self.model.new_constant(0)
            else:
                total_vol = sum(flow_vars)
                div_result = self.model.new_int_var(0, 1440, f"serv_wh_{loc.id}_v{v.id}")
                speed = int(loc.handling_speed) if loc.handling_speed > 1 else 1
                self.model.add(total_vol <= div_result * speed)
                self.model.add(total_vol > (div_result - 1) * speed)
                expr = div_result
        else:
            expr = self.model.new_constant(0)

        self.var_manager.service_time_cache[key] = expr
        return expr

    def _add_routing_constraints(self):
        allowed_pairs = self.pruner.get_allowed_pairs()

        for v in self.scenario.vehicles:
            is_used = self.var_manager.get_vehicle_used_var(v.id)

            # 0. Запрет петель
            for loc in self.scenario.all_locations:
                x_var = self.var_manager.get_routing_var(v.id, loc.id, loc.id)
                if x_var is not None and not isinstance(x_var, int):
                    self.model.add(x_var == 0)

            # --- ФИКС БАГА 4 (Единая математика маршрута, без тормозов солвера) ---
            end_flags = []
            start_arcs = []

            for loc in self.scenario.all_locations:
                inc = [self.var_manager.get_routing_var(v.id, i, loc.id) for i, j in allowed_pairs if j == loc.id]
                out = [self.var_manager.get_routing_var(v.id, loc.id, j) for i, j in allowed_pairs if i == loc.id]
                inc = [x for x in inc if x is not None]
                out = [x for x in out if x is not None]

                is_visited = self.var_manager.get_is_visited_var(v.id, loc.id)
                is_end = self.var_manager.get_is_end_var(v.id, loc.id)
                if is_visited is None or is_end is None: continue

                self.model.add(sum(inc) <= 1)
                self.model.add(sum(out) <= 1)

                if isinstance(loc, Store):
                    # Жесткая математика: визит = въезд. Конец пути = въехали, но не выехали.
                    self.model.add(is_visited == sum(inc))
                    self.model.add(sum(inc) >= sum(out))
                    self.model.add(is_end == sum(inc) - sum(out))
                else:
                    self.model.add_max_equality(is_visited, [sum(inc), sum(out)])
                    self.model.add(is_end == sum(inc))
                    start_arcs.extend(out)

                end_flags.append(is_end)

            if end_flags:
                self.model.add(sum(end_flags) == is_used)
            if start_arcs:
                self.model.add(sum(start_arcs) == is_used)

            # 3. Расстояние
            path_dist_terms = []
            for i_id, j_id in allowed_pairs:
                x_var = self.var_manager.get_routing_var(v.id, i_id, j_id)
                if x_var is not None:
                    dist = self.scenario.network.distance_matrix[i_id][j_id]
                    path_dist_terms.append(x_var * int(dist * 100))

            total_dist_scaled = self.model.new_int_var(0, 1_000_000 * 100, f'dist_sc_v{v.id}')
            self.model.add(total_dist_scaled == sum(path_dist_terms)).OnlyEnforceIf(is_used)
            self.model.add(total_dist_scaled == 0).OnlyEnforceIf(is_used.Not())
            self.model.add_division_equality(self.var_manager.get_total_dist_var(v.id), total_dist_scaled, 100)

    def _add_load_flow_constraints(self):
        K = self.scenario.units_per_crate

        # ФЛАГИ ДИАГНОСТИКИ (включай по одному!)

        CHECK_CONSERVATION = True  # Закон сохранения (въехало - выгрузил = выехало)
        CHECK_CONVERSION = True  # Математика "Штуки -> Ящики" (Ceil)
        CHECK_EMPTY_END = False  # Обязательно пустеть в конце

        for v in self.scenario.vehicles:
            is_unit_delivery = (v.category.lower() == "штуки")
            max_capacity_val = (v.capacity * K) #if is_unit_delivery else v.capacity
            # Считаем сумму именно ЯЩИКОВ (те, что округлены вверх в CHECK_CONVERSION)
            total_crates_by_vehicle = sum(
                self.var_manager.get_delivery_crates_var(v.id, s.id, b.id)
                for s in self.scenario.stores
                for b in self.scenario.brands
                if self.var_manager.get_delivery_crates_var(v.id, s.id, b.id) is not None
            )

            # Оставляем сумму ШТУК для баланса
            total_units_by_vehicle = sum(
                self.var_manager.get_delivery_var(v.id, s.id, b.id)
                for s in self.scenario.stores
                for b in self.scenario.brands
                if self.var_manager.get_delivery_var(v.id, s.id, b.id) is not None
            )

            is_used = self.var_manager.get_vehicle_used_var(v.id)

            # Жестко запрещаем суммарно развезти больше, чем влезает в кузов!
            # Проверяем вместимость по ЯЩИКАМ (лимит 500)
            self.model.add(total_crates_by_vehicle <= v.capacity).OnlyEnforceIf(is_used)

            # А обнуление при простое оставляем по ШТУКАМ
            self.model.add(total_units_by_vehicle == 0).OnlyEnforceIf(is_used.Not())
            for wh in self.scenario.warehouses:
                 is_at_wh = self.var_manager.get_is_visited_var(v.id, wh.id)
                 if is_at_wh is not None:
                     # На складе выездной объем (at_point) равен всей доставке за день
                     # Машина выезжает со склада, имея в кузове сумму всех ШТУК (для баланса в точках)
                     self.model.add(
                         self.var_manager.get_load_at_point_var(v.id, wh.id) == total_units_by_vehicle).OnlyEnforceIf(
                         is_at_wh)
            # 1. ПЕРЕДАЧА ПОТОКА (Связь между точками)
            # Если это не работает - проблема в индексах x_kij
            if CHECK_CONSERVATION:
                for i in self.scenario.all_locations:
                    for j in self.scenario.all_locations:
                        x_kij = self.var_manager.get_routing_var(v.id, i.id, j.id)
                        if x_kij is None: continue
                        self.model.add(
                            self.var_manager.get_load_arriving_var(v.id, j.id) ==
                            self.var_manager.get_load_at_point_var(v.id, i.id)
                        ).OnlyEnforceIf(x_kij)

            # 2. ЛОКАЛЬНЫЙ БАЛАНС В ТОЧКЕ
            for loc in self.scenario.all_locations:
                is_loc_visited = self.var_manager.get_is_visited_var(v.id, loc.id)
                if is_loc_visited is None: continue
                arr_var = self.var_manager.get_load_arriving_var(v.id, loc.id)
                dep_var = self.var_manager.get_load_at_point_var(v.id, loc.id)

                # Принудительно зажимаем значения переменных в физические рамки:
                self.model.add(arr_var >= 0)
                self.model.add(arr_var <= max_capacity_val)
                self.model.add(dep_var >= 0)
                self.model.add(dep_var <= max_capacity_val)
                # Конвертация штук в ящики
                if CHECK_CONVERSION and isinstance(loc, Store):
                    for b in self.scenario.brands:
                        units_var = self.var_manager.get_delivery_var(v.id, loc.id, b.id)
                        crates_var = self.var_manager.get_delivery_crates_var(v.id, loc.id, b.id)
                        if units_var is not None and crates_var is not None:
                            self.model.add(crates_var * K >= units_var)
                            self.model.add(crates_var * K <= units_var + K - 1)

                # Расчет потребления
                consumed = sum(self.var_manager.get_delivery_var(v.id, loc.id, b.id) for b in self.scenario.brands)
                # Временно игнорируем pickup для простоты

                if CHECK_CONSERVATION:
                    self.model.add(
                        dep_var ==
                        arr_var - consumed
                    ).OnlyEnforceIf(is_loc_visited)
                    self.model.add(dep_var == arr_var).OnlyEnforceIf(is_loc_visited.Not())


                if CHECK_EMPTY_END and isinstance(loc, Store):
                    is_end_here = self.var_manager.get_is_end_var(v.id, loc.id)
                    self.model.add(dep_var == 0).OnlyEnforceIf(is_end_here)

    def _add_time_window_constraints(self):


        for v in self.scenario.vehicles:
            is_used = self.var_manager.get_vehicle_used_var(v.id)

            for loc in self.scenario.all_locations:
                arr_var = self.var_manager.get_arrival_var(v.id, loc.id)
                is_visited = self.var_manager.get_is_visited_var(v.id, loc.id)
                if arr_var is not None and is_visited is not None:
                    self.model.add(arr_var == 0).OnlyEnforceIf(is_visited.Not())

            for wh in self.scenario.warehouses:
                is_starting_wh = self.model.new_bool_var(f"is_start_v{v.id}_wh{wh.id}")
                outgoing = [self.var_manager.get_routing_var(v.id, wh.id, j.id)
                            for j in self.scenario.all_locations
                            if self.var_manager.get_routing_var(v.id, wh.id, j.id) is not None]
                if outgoing:
                    self.model.add_bool_or(outgoing).OnlyEnforceIf(is_starting_wh)
                    self.model.add(sum(outgoing) == 0).OnlyEnforceIf(is_starting_wh.Not())
                    arr_wh = self.var_manager.get_arrival_var(v.id, wh.id)
                    if arr_wh is not None:
                        self.model.add(arr_wh == self.var_manager.get_shift_start_var(v.id)).OnlyEnforceIf(
                            is_starting_wh)
                else:
                    self.model.add(is_starting_wh == 0)

            for i in self.scenario.all_locations:
                arr_i = self.var_manager.get_arrival_var(v.id, i.id)
                if arr_i is None: continue
                service_i = self._get_service_time_expr(v, i)

                for j in self.scenario.all_locations:
                    x_kij = self.var_manager.get_routing_var(v.id, i.id, j.id)
                    if x_kij is None: continue

                    arr_j = self.var_manager.get_arrival_var(v.id, j.id)
                    if arr_j is None: continue

                    travel_ij = int(self.scenario.network.time_matrix[i.id][j.id])
                    self.model.add(arr_j >= arr_i + service_i + travel_ij).OnlyEnforceIf(x_kij)

                    loc_j = self.location_by_id[j.id]
                    if isinstance(loc_j, Store):
                        self.model.add(arr_j >= loc_j.time_start).OnlyEnforceIf(x_kij)
                        self.model.add(arr_j <= loc_j.time_end).OnlyEnforceIf(x_kij)

            # РАСЧЕТ ОБЩЕГО ВРЕМЕНИ СМЕНЫ (Используем единый is_visited)
            s_start = self.var_manager.get_shift_start_var(v.id)
            s_end = self.var_manager.get_shift_end_var(v.id)
            s_total = self.var_manager.get_total_time_var(v.id)

            for loc in self.scenario.all_locations:
                arr_loc = self.var_manager.get_arrival_var(v.id, loc.id)
                is_visited = self.var_manager.get_is_visited_var(v.id, loc.id)
                if arr_loc is None or is_visited is None: continue

                self.model.add(s_start <= arr_loc).OnlyEnforceIf(is_visited)
                service = self._get_service_time_expr(v, loc)
                self.model.add(s_end >= arr_loc + service).OnlyEnforceIf(is_visited)

            self.model.add(s_total == s_end - s_start).OnlyEnforceIf(is_used)
            self.model.add(s_total == 0).OnlyEnforceIf(is_used.Not())
            self.model.add(s_start == 0).OnlyEnforceIf(is_used.Not())

    def _add_demand_satisfaction_constraints(self):
        for store in self.scenario.stores:
            # 1. Список визитов машин в этот магазин
            visits = [self.var_manager.get_is_visited_var(v.id, store.id) for v in self.scenario.vehicles]

            # ПРАВИЛО: В один магазин может приехать МАКСИМУМ 2 машины (как вы и хотели)
            self.model.add(sum(visits) == 1 )

            # ПРАВИЛО: Магазин ОБЯЗАН быть посещен хотя бы одной машиной
            #self.model.add(sum(visits) >= 1)

            # --- ЗАКРЫВАЕМ ДЫРУ ---
            # add_max_equality делает жесткую связь:
            # is_visited = 1, если в массиве visits есть хотя бы одна 1.
            # is_visited = 0, если в массиве visits все нули.
            is_visited = self.model.new_bool_var(f"any_visit_s{store.id}")
            self.model.add_max_equality(is_visited, visits)

            # Так как мы выше жестко сказали sum(visits) >= 1,
            # is_visited ТЕПЕРЬ ВСЕГДА БУДЕТ РАВЕН 1. Никаких нулей!

            for brand in self.scenario.brands:
                brand_demand_dict = store.demands.get(brand.id, {})
                base_demand = next(iter(brand_demand_dict.values()), 0) if brand_demand_dict else 0

                for v in self.scenario.vehicles:
                    del_var = self.var_manager.get_delivery_var(v.id, store.id, brand.id)
                    vis_var = self.var_manager.get_is_visited_var(v.id, store.id)

                    if del_var is not None and vis_var is not None:
                        self.model.add(del_var >= 0)
                        # ПРАВИЛО: Если машина не заехала -> доставка этой машины = 0
                        self.model.add(del_var == 0).OnlyEnforceIf(vis_var.Not())
                        self.model.add(del_var <= base_demand).OnlyEnforceIf(vis_var)

                # Сколько реально привезли в этот магазин всеми машинами
                total_delivered_to_store = sum(
                    self.var_manager.get_delivery_var(v.id, store.id, brand.id)
                    for v in self.scenario.vehicles
                    if self.var_manager.get_delivery_var(v.id, store.id, brand.id) is not None
                )

                if base_demand > 0:
                    # Если заехали: привезти от 80% до 100%
                    self.model.add(total_delivered_to_store >= int(base_demand * 0.8)).OnlyEnforceIf(is_visited)
                    self.model.add(total_delivered_to_store <= base_demand).OnlyEnforceIf(is_visited)

                    # Если не заехали: доставка 0
                    self.model.add(total_delivered_to_store == 0).OnlyEnforceIf(is_visited.Not())
                else:
                    self.model.add(total_delivered_to_store == 0)
        all_store_visits = []
        for store in self.scenario.stores:
            for v in self.scenario.vehicles:
                vis = self.var_manager.get_is_visited_var(v.id, store.id)
                if vis is not None:
                    all_store_visits.append(vis)

        # Требуем, чтобы в сумме по всему городу было обслужено ХОТЯ БЫ 10 магазинов
        # (Если 10 не сработает - поставьте 5 или 1, чтобы нащупать предел)
        if all_store_visits:
            self.model.add(sum(all_store_visits) >= 52)

    def _add_vehicle_activity_linkage_constraints(self):
        # --- ФИКС ЛЯМБДЫ 0.45: Машина используется ТОЛЬКО если посетила >= 1 магазина ---
        for v in self.scenario.vehicles:
            is_used = self.var_manager.get_vehicle_used_var(v.id)

            store_visits = []
            for store in self.scenario.stores:
                vis = self.var_manager.get_is_visited_var(v.id, store.id)
                if vis is not None:
                    store_visits.append(vis)

            if store_visits:
                self.model.add(sum(store_visits) >= 1).OnlyEnforceIf(is_used)
                self.model.add(sum(store_visits) == 0).OnlyEnforceIf(is_used.Not())
            else:
                self.model.add(is_used == 0)

    def _add_warehouse_constraints(self):
        """
        Реализует ограничения для складов: индикатор использования и Reservoir.
        """
        for wh in self.scenario.warehouses:
            wh_active = self.var_manager.get_wh_active_var(wh.id)

            # --- ПРАВКА 1: ЛОГИКА ДЛЯ ЗАВОДОВ (Упрощенная) ---
            if wh.is_factory:
                wh_max_vol = self.var_manager.get_wh_max_vol_var(wh.id)
                # Сумма всего, что выгрузили/загрузили на заводе
                total_flow = sum(
                    self.var_manager.get_delivery_var(v.id, wh.id, b.id) for v in self.scenario.vehicles for b in
                    self.scenario.brands) + \
                             sum(self.var_manager.get_pickup_var(v.id, wh.id, b.id) for v in self.scenario.vehicles for
                                 b in self.scenario.brands)

                self.model.add(wh_max_vol == total_flow).OnlyEnforceIf(wh_active)
                self.model.add(wh_max_vol == 0).OnlyEnforceIf(wh_active.Not())

                # Завод активен, если хоть одна машина оттуда выехала
                start_arcs = []
                for v in self.scenario.vehicles:
                    for j_id in self.var_manager.reachable_locs:
                        arc = self.var_manager.get_routing_var(v.id, wh.id, j_id)
                        if arc is not None:  # ИСПРАВЛЕНО
                            start_arcs.append(arc)
                if start_arcs:
                    self.model.add_bool_or(start_arcs).OnlyEnforceIf(wh_active)
                    for a in start_arcs: self.model.add(wh_active == 1).OnlyEnforceIf(a)
                else:
                    self.model.add(wh_active == 0)
                continue  # Завод закончили, идем к следующему складу

            # --- ПРАВКА 2: ЛОГИКА ДЛЯ РЦ (Склады с Reservoir) ---
            all_visits = []
            for v in self.scenario.vehicles:
                is_v_at_wh = self.var_manager.get_wh_visit_active_flag(wh.id, v.id)
                all_visits.append(is_v_at_wh)

                # Исходящие дуги как признак визита
                outgoing = [self.var_manager.get_routing_var(v.id, wh.id, j_id)
                            for j_id in self.var_manager.reachable_locs
                            if self.var_manager.get_routing_var(v.id, wh.id, j_id) is not None]

                if outgoing:
                    self.model.add_bool_or(outgoing).OnlyEnforceIf(is_v_at_wh)
                    for arc in outgoing: self.model.add(is_v_at_wh == 1).OnlyEnforceIf(arc)
                else:
                    self.model.add(is_v_at_wh == 0)

                # Reservoir: Интервалы (Только для РЦ!)
                interval = self.var_manager.get_wh_visit_interval_var(wh.id, v.id)
                arrival = self.var_manager.get_arrival_var(v.id, wh.id)
                service = self._get_service_time_expr(v, wh)
                self.model.add(interval.StartExpr() == arrival).OnlyEnforceIf(is_v_at_wh)
                self.model.add(interval.SizeExpr() == service).OnlyEnforceIf(is_v_at_wh)

            # Связь визитов машин с активностью РЦ
            if all_visits:
                self.model.add_bool_or(all_visits).OnlyEnforceIf(wh_active)
                for v_vis in all_visits: self.model.add(wh_active == 1).OnlyEnforceIf(v_vis)
            else:
                self.model.add(wh_active == 0)

            # Баланс Reservoir (Только для РЦ)
            for b in self.scenario.brands:
                times, demands, actives = [], [], []
                initial_stock = wh.initial_stock.get(b.id, 0)
                if initial_stock > 0:
                    times.append(0)
                    demands.append(initial_stock)
                    actives.append(self.model.new_constant(1))

                for v_obj in self.scenario.vehicles:
                    del_v = self.var_manager.get_delivery_var(v_obj.id, wh.id, b.id)
                    pick_v = self.var_manager.get_pickup_var(v_obj.id, wh.id, b.id)
                    if del_v is None or pick_v is None: continue

                    change = self.var_manager.get_wh_stock_change_per_visit_var(wh.id, b.id, v_obj.id)
                    self.model.add(change == del_v - pick_v)
                    times.append(self.var_manager.get_arrival_var(v_obj.id, wh.id))
                    demands.append(change)
                    actives.append(self.var_manager.get_wh_visit_active_flag(wh.id, v_obj.id))

                if times:
                    self.model.add_reservoir_constraint_with_active(times, demands, actives, 0, 10_000_000)

            # Объем РЦ (w_iv)
            wh_max_vol = self.var_manager.get_wh_max_vol_var(wh.id)
            if self.warehouse_cost_mode == WarehouseCostMode.PEAK_INPUT:
                total_delivered = sum(self.var_manager.get_delivery_var(v.id, wh.id, b.id)
                                      for v in self.scenario.vehicles for b in self.scenario.brands)
                self.model.add(wh_max_vol == sum(wh.initial_stock.values()) + total_delivered).OnlyEnforceIf(wh_active)

            self.model.add(wh_max_vol == 0).OnlyEnforceIf(wh_active.Not())

        # Запрет возврата товара в магазинах (для всех складов)
        for store in self.scenario.stores:
            for v_obj in self.scenario.vehicles:
                for b in self.scenario.brands:
                    p_var = self.var_manager.get_pickup_var(v_obj.id, store.id, b.id)
                    if p_var is not None: self.model.add(p_var == 0)

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